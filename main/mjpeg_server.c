/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mjpeg_server.h"
#include "example_config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "image_processor.h"
#include "esp_jpeg_enc.h"
#include "face_detect_task.h"

static const char *TAG = "video_server";

#define MAX_STREAM_CLIENTS 4

#define SSE_MAX_CLIENTS 4
typedef struct {
    httpd_req_t *req;  // async request handle; NULL = slot free
} sse_client_t;

static sse_client_t      s_sse_clients[SSE_MAX_CLIENTS];
static SemaphoreHandle_t s_sse_mutex = NULL;

typedef struct {
    SemaphoreHandle_t frame_ready;
    bool active;
} stream_client_t;

// ---------------------------------------------------------------------------
// Private state
// ---------------------------------------------------------------------------
static uint8_t          *s_shared_buf   = NULL;
static size_t            s_shared_size  = 0;
static size_t            s_buf_capacity = 0;
static SemaphoreHandle_t s_buf_mutex    = NULL;
static httpd_handle_t    s_server       = NULL;
static stream_client_t   s_clients[MAX_STREAM_CLIENTS] = {0};

// ---------------------------------------------------------------------------
// HTML viewer page (Premium Dashboard using JMuxer for H.264 decoding)
// ---------------------------------------------------------------------------
static const char STREAM_PAGE_HTML[] =
    "<!DOCTYPE html>"
    "<html lang='de'>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32-P4 H.264 Camera Dashboard</title>"
    "<link href='https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600&display=swap' rel='stylesheet'>"
    "<script src='https://cdn.jsdelivr.net/npm/jmuxer@2.0.6/dist/jmuxer.min.js'></script>"
    "<style>"
    ":root {"
    "  --bg-color: #08090c;"
    "  --card-bg: rgba(255, 255, 255, 0.03);"
    "  --card-border: rgba(255, 255, 255, 0.06);"
    "  --text-primary: #f3f4f6;"
    "  --text-secondary: #9ca3af;"
    "  --accent: #4f46e5;"
    "  --accent-glow: rgba(79, 70, 229, 0.4);"
    "  --success: #10b981;"
    "  --danger: #ef4444;"
    "}"
    "* { box-sizing: border-box; margin: 0; padding: 0; }"
    "body {"
    "  background-color: var(--bg-color);"
    "  background-image: radial-gradient(circle at 50% 50%, #151622 0%, #08090c 100%);"
    "  color: var(--text-primary);"
    "  font-family: 'Outfit', sans-serif;"
    "  min-height: 100vh;"
    "  display: flex;"
    "  flex-direction: column;"
    "  align-items: center;"
    "  padding: 20px;"
    "}"
    ".container {"
    "  max-width: 1000px;"
    "  width: 100%;"
    "  display: flex;"
    "  flex-direction: column;"
    "  gap: 20px;"
    "}"
    "header {"
    "  display: flex;"
    "  justify-content: space-between;"
    "  align-items: center;"
    "  padding: 10px 0;"
    "  border-bottom: 1px solid var(--card-border);"
    "}"
    "h1 { font-size: 1.8rem; font-weight: 600; background: linear-gradient(to right, #a5b4fc, #6366f1); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }"
    ".status-badge {"
    "  display: flex;"
    "  align-items: center;"
    "  gap: 8px;"
    "  background: var(--card-bg);"
    "  border: 1px solid var(--card-border);"
    "  padding: 6px 14px;"
    "  border-radius: 20px;"
    "  font-size: 0.85rem;"
    "  font-weight: 600;"
    "}"
    ".status-dot {"
    "  width: 8px;"
    "  height: 8px;"
    "  border-radius: 50%;"
    "  background-color: var(--success);"
    "  box-shadow: 0 0 8px var(--success);"
    "  animation: pulse 2s infinite;"
    "}"
    "@keyframes pulse {"
    "  0% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(16, 185, 129, 0.7); }"
    "  70% { transform: scale(1); box-shadow: 0 0 0 8px rgba(16, 185, 129, 0); }"
    "  100% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(16, 185, 129, 0); }"
    "}"
    ".grid {"
    "  display: grid;"
    "  grid-template-columns: 1fr 300px;"
    "  gap: 20px;"
    "}"
    "@media(max-width: 768px) {"
    "  .grid { grid-template-columns: 1fr; }"
    "}"
    ".card {"
    "  background: var(--card-bg);"
    "  backdrop-filter: blur(12px);"
    "  -webkit-backdrop-filter: blur(12px);"
    "  border: 1px solid var(--card-border);"
    "  border-radius: 16px;"
    "  padding: 20px;"
    "  box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.3);"
    "}"
    ".view-card {"
    "  display: flex;"
    "  flex-direction: column;"
    "  align-items: center;"
    "  justify-content: center;"
    "  position: relative;"
    "  min-height: 350px;"
    "}"
    ".stream-wrapper {"
    "  background: #000;"
    "  border-radius: 12px;"
    "  overflow: hidden;"
    "  border: 1px solid rgba(255, 255, 255, 0.1);"
    "  display: flex;"
    "  align-items: center;"
    "  justify-content: center;"
    "  box-shadow: inset 0 0 20px rgba(0,0,0,0.8);"
    "  position: relative;"
    "}"
    "#player {"
    "  display: block;"
    "  transition: transform 0.2s ease, width 0.2s ease, height 0.2s ease;"
    "}"
    ".control-panel {"
    "  margin-top: 20px;"
    "  width: 100%;"
    "  display: flex;"
    "  flex-wrap: wrap;"
    "  gap: 12px;"
    "  justify-content: center;"
    "}"
    "button {"
    "  padding: 10px 20px;"
    "  font-size: 0.9rem;"
    "  font-weight: 600;"
    "  border: 1px solid var(--card-border);"
    "  border-radius: 8px;"
    "  cursor: pointer;"
    "  color: var(--text-primary);"
    "  background: var(--card-bg);"
    "  transition: all 0.2s ease;"
    "  font-family: inherit;"
    "}"
    "button:hover {"
    "  background: rgba(255, 255, 255, 0.08);"
    "  border-color: rgba(255, 255, 255, 0.2);"
    "  transform: translateY(-1px);"
    "}"
    "button.primary {"
    "  background: var(--accent);"
    "  border-color: var(--accent);"
    "  box-shadow: 0 4px 12px var(--accent-glow);"
    "}"
    "button.primary:hover {"
    "  background: #4338ca;"
    "  box-shadow: 0 6px 16px var(--accent-glow);"
    "}"
    "button.danger {"
    "  background: var(--danger);"
    "  border-color: var(--danger);"
    "}"
    "button.danger:hover {"
    "  background: #dc2626;"
    "}"
    "button.active {"
    "  background: rgba(255, 255, 255, 0.15);"
    "  border-color: rgba(255, 255, 255, 0.4);"
    "}"
    ".stats-list {"
    "  display: flex;"
    "  flex-direction: column;"
    "  gap: 14px;"
    "}"
    ".stat-item {"
    "  display: flex;"
    "  justify-content: space-between;"
    "  align-items: center;"
    "  padding-bottom: 10px;"
    "  border-bottom: 1px solid rgba(255, 255, 255, 0.05);"
    "}"
    ".stat-item:last-child {"
    "  border-bottom: none;"
    "  padding-bottom: 0;"
    "}"
    ".stat-label {"
    "  color: var(--text-secondary);"
    "  font-size: 0.9rem;"
    "}"
    ".stat-value {"
    "  font-weight: 600;"
    "  font-size: 0.95rem;"
    "}"
    ".badge {"
    "  padding: 3px 8px;"
    "  border-radius: 6px;"
    "  font-size: 0.75rem;"
    "  background: rgba(255, 255, 255, 0.08);"
    "  border: 1px solid rgba(255, 255, 255, 0.1);"
    "}"
    ".badge.accent {"
    "  background: rgba(99, 102, 241, 0.15);"
    "  border-color: rgba(99, 102, 241, 0.3);"
    "  color: #a5b4fc;"
    "}"
    "footer {"
    "  text-align: center;"
    "  padding: 20px 0;"
    "  font-size: 0.8rem;"
    "  color: var(--text-secondary);"
    "  border-top: 1px solid var(--card-border);"
    "  margin-top: auto;"
    "}"
    ".sidebar {"
    "  display: flex;"
    "  flex-direction: column;"
    "  gap: 20px;"
    "}"
    ".input-group {"
    "  display: flex;"
    "  flex-direction: column;"
    "  gap: 8px;"
    "  margin-bottom: 16px;"
    "}"
    ".input-group label {"
    "  font-size: 0.85rem;"
    "  color: var(--text-secondary);"
    "  font-weight: 500;"
    "}"
    ".input-row {"
    "  display: flex;"
    "  gap: 8px;"
    "}"
    ".text-input {"
    "  background: rgba(255, 255, 255, 0.05);"
    "  border: 1px solid var(--card-border);"
    "  border-radius: 8px;"
    "  padding: 10px 12px;"
    "  color: var(--text-primary);"
    "  font-family: inherit;"
    "  font-size: 0.9rem;"
    "  outline: none;"
    "  transition: all 0.2s ease;"
    "  flex-grow: 1;"
    "}"
    ".text-input:focus {"
    "  border-color: var(--accent);"
    "  box-shadow: 0 0 8px var(--accent-glow);"
    "  background: rgba(255, 255, 255, 0.08);"
    "}"
    ".number-input {"
    "  width: 80px;"
    "  text-align: center;"
    "}"
    ".project-path-box {"
    "  background: rgba(255, 255, 255, 0.02);"
    "  border: 1px solid rgba(255, 255, 255, 0.04);"
    "  padding: 10px 12px;"
    "  border-radius: 8px;"
    "  font-size: 0.85rem;"
    "  margin-bottom: 16px;"
    "  word-break: break-all;"
    "}"
    ".radio-group {"
    "  display: flex;"
    "  flex-direction: column;"
    "  gap: 8px;"
    "  margin-top: 10px;"
    "  max-height: 180px;"
    "  overflow-y: auto;"
    "  padding-right: 4px;"
    "}"
    ".cat-radio-item {"
    "  display: flex;"
    "  align-items: center;"
    "  gap: 10px;"
    "  padding: 8px 12px;"
    "  background: rgba(255, 255, 255, 0.03);"
    "  border: 1px solid var(--card-border);"
    "  border-radius: 8px;"
    "  cursor: pointer;"
    "  transition: all 0.2s ease;"
    "}"
    ".cat-radio-item:hover {"
    "  background: rgba(255, 255, 255, 0.06);"
    "  border-color: rgba(255, 255, 255, 0.1);"
    "}"
    ".cat-radio-item input[type='radio'] {"
    "  appearance: none;"
    "  -webkit-appearance: none;"
    "  width: 16px;"
    "  height: 16px;"
    "  border: 2px solid var(--text-secondary);"
    "  border-radius: 50%;"
    "  outline: none;"
    "  cursor: pointer;"
    "  display: flex;"
    "  align-items: center;"
    "  justify-content: center;"
    "  transition: all 0.2s ease;"
    "}"
    ".cat-radio-item input[type='radio']:checked {"
    "  border-color: var(--accent);"
    "}"
    ".cat-radio-item input[type='radio']:checked::before {"
    "  content: '';"
    "  width: 8px;"
    "  height: 8px;"
    "  background-color: var(--accent);"
    "  border-radius: 50%;"
    "}"
    ".cat-radio-item label {"
    "  cursor: pointer;"
    "  flex-grow: 1;"
    "  font-size: 0.9rem;"
    "  user-select: none;"
    "}"
    ".status-msg {"
    "  margin-top: 12px;"
    "  font-size: 0.85rem;"
    "  padding: 8px 12px;"
    "  border-radius: 6px;"
    "  display: none;"
    "  animation: fadeIn 0.3s ease;"
    "}"
    "@keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }"
    "</style>"
    "</head>"
    "<body>"
    "  <div class='container'>"
    "    <header>"
    "      <h1>ESP32-P4 H.264 Camera</h1>"
    "      <div class='status-badge'>"
    "        <div class='status-dot' id='status-dot'></div>"
    "        <span id='status-text'>Offline</span>"
    "      </div>"
    "    </header>"
    "    <div class='grid'>"
    "      <div class='card view-card'>"
    "        <div class='stream-wrapper' id='stream-wrapper'>"
    "          <div id='player-container' style='position:relative;display:inline-block;'>"
    "            <video id='player' autoplay muted playsinline></video>"
    "            <div id='face-box' style='position:absolute;border:3px solid #38bdf8;border-radius:6px;pointer-events:none;display:none;transition:all 0.05s ease;box-shadow:0 0 8px rgba(56,189,248,0.5);'></div>"
    "          </div>"
    "        </div>"
    "        <div class='control-panel'>"
    "          <button id='btn-start' class='primary' onclick='startStream()'>Start</button>"
    "          <button id='btn-stop' class='danger' onclick='stopStream()'>Stop</button>"
    "          <button id='btn-scale' onclick='toggleScaling()'>Scale: 2x</button>"
    "        </div>"
    "      </div>"
    "      <div class='sidebar'>"
    "        <div class='card'>"
    "          <h3 style='margin-bottom:16px;font-weight:600;font-size:1.1rem;'>Device Info</h3>"
    "          <div class='stats-list'>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>Resolution</span>"
    "              <span class='stat-value'><span class='badge accent' id='val-res'>-</span></span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>Uptime</span>"
    "              <span class='stat-value' id='val-uptime'>-</span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>WiFi Signal</span>"
    "              <span class='stat-value' id='val-wifi'>-</span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>Free SRAM</span>"
    "              <span class='stat-value' id='val-sram'>-</span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>Free SPIRAM</span>"
    "              <span class='stat-value' id='val-spiram'>-</span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>CPU 0 Load</span>"
    "              <span class='stat-value' id='val-cpu0'>-</span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>CPU 1 Load</span>"
    "              <span class='stat-value' id='val-cpu1'>-</span>"
    "            </div>"
    "          </div>"
    "        </div>"
    "        <div class='card'>"
    "          <h3 style='margin-bottom:16px;font-weight:600;font-size:1.1rem;background:linear-gradient(to right, #38bdf8, #0284c7);-webkit-background-clip:text;-webkit-text-fill-color:transparent;'>Gesichtserkennung</h3>"
    "          <div class='stats-list'>"
    "            <div class='stat-item' style='flex-direction:column;align-items:flex-start;gap:6px;'>"
    "              <span class='stat-label'>Gesicht erkannt</span>"
    "              <span class='stat-value' id='val-gesture' style='font-size:1.8rem;color:#38bdf8;font-weight:700;transition:all 0.2s ease;'>no_face</span>"
    "            </div>"
    "            <div class='stat-item'>"
    "              <span class='stat-label'>Konfidenz</span>"
    "              <span class='stat-value' id='val-gesture-score'>0.0%</span>"
    "            </div>"
    "          </div>"
    "        </div>"
    "        <div class='card'>"
    "          <h3 style='margin-bottom:16px;font-weight:600;font-size:1.1rem;'>Dataset Collector</h3>"
    "          <div class='input-group'>"
    "            <label for='input-project-name'>Projektname</label>"
    "            <input type='text' id='input-project-name' class='text-input' value='TransferProject' placeholder='z.B. Dataset_1' oninput='updateDownloadButtonText()'>"
    "          </div>"
    "          <div class='input-group'>"
    "            <label for='num-categories'>Anzahl Kategorien</label>"
    "            <div class='input-row'>"
    "              <input type='number' id='num-categories' class='text-input number-input' value='3' min='1' max='20'>"
    "              <button id='btn-init-project' class='primary' style='flex-grow:1;' onclick='initializeProject()'>Projekt erstellen</button>"
    "            </div>"
    "          </div>"
    "          <div id='categories-section' style='display:none;margin-top:16px;'>"
    "            <h4 style='font-size:0.9rem;font-weight:600;margin-bottom:8px;color:var(--text-secondary);'>Aktuelle Kategorie</h4>"
    "            <div id='categories-container' class='radio-group'></div>"
    "            <button id='btn-download' class='primary' style='width:100%;margin-top:16px;' onclick='downloadSnapshot()'>Download</button>"
    "          </div>"
    "          <div id='status-message' class='status-msg'></div>"
    "        </div>"
    "      </div>"
    "    </div>"
    "    <footer>"
    "      ESP32-P4 MIPI CSI Video Server &bull; Powered by ESP-IDF v5.5.4"
    "    </footer>"
    "  </div>"
    "  <script>"
    "    var player = document.getElementById('player');"
    "    var btnScale = document.getElementById('btn-scale');"
    "    var statusDot = document.getElementById('status-dot');"
    "    var statusText = document.getElementById('status-text');"
    "    var scaleFactor = 2;"
    "    var actualWidth = 224;"
    "    var actualHeight = 224;"
    "    var jmuxer = null;"
    "    var controller = null;"
    "    var projectInitialized = false;"
    "    var categoryCounters = {};"
    "    function updateImageDimensions() {"
    "      player.style.width = (actualWidth * scaleFactor) + 'px';"
    "      player.style.height = (actualHeight * scaleFactor) + 'px';"
    "    }"
    "    function showStatus(text, type) {"
    "      var statusDiv = document.getElementById('status-message');"
    "      statusDiv.textContent = text;"
    "      statusDiv.style.display = 'block';"
    "      if (type === 'success') {"
    "        statusDiv.style.background = 'rgba(16, 185, 129, 0.15)';"
    "        statusDiv.style.border = '1px solid rgba(16, 185, 129, 0.3)';"
    "        statusDiv.style.color = '#a7f3d0';"
    "      } else if (type === 'error') {"
    "        statusDiv.style.background = 'rgba(239, 68, 68, 0.15)';"
    "        statusDiv.style.border = '1px solid rgba(239, 68, 68, 0.3)';"
    "        statusDiv.style.color = '#fca5a5';"
    "      } else {"
    "        statusDiv.style.background = 'rgba(255, 255, 255, 0.05)';"
    "        statusDiv.style.border = '1px solid rgba(255, 255, 255, 0.1)';"
    "        statusDiv.style.color = 'var(--text-secondary)';"
    "      }"
    "    }"
    "    function updateDownloadButtonText() {"
    "      var activeRadio = document.querySelector('input[name=\"current-category\"]:checked');"
    "      var btn = document.getElementById('btn-download');"
    "      if (activeRadio && btn) {"
    "        var cat = activeRadio.value;"
    "        var nextNum = categoryCounters[cat] || 1;"
    "        var projName = document.getElementById('input-project-name').value.trim().replace(/[^a-zA-Z0-9_-]/g, '') || 'Projekt';"
    "        btn.textContent = 'Download: ' + projName + '_' + cat + '_Bild_' + nextNum + '.jpg';"
    "      }"
    "    }"
    "    async function initializeProject() {"
    "      var projNameInput = document.getElementById('input-project-name');"
    "      var projName = projNameInput.value.trim().replace(/[^a-zA-Z0-9_-]/g, '');"
    "      if (!projName) {"
    "        showStatus('Bitte einen gültigen Projektnamen eingeben (nur Buchstaben, Zahlen, _ und -).', 'error');"
    "        return;"
    "      }"
    "      var numCatsInput = document.getElementById('num-categories');"
    "      var n = parseInt(numCatsInput.value);"
    "      if (isNaN(n) || n < 1 || n > 20) {"
    "        showStatus('Kategorienanzahl muss zwischen 1 und 20 liegen.', 'error');"
    "        return;"
    "      }"
    "      categoryCounters = {};"
    "      var container = document.getElementById('categories-container');"
    "      container.innerHTML = '';"
    "      for (var i = 1; i <= n; i++) {"
    "        var catName = 'cat_' + i;"
    "        categoryCounters[catName] = 1;"
    "        var div = document.createElement('div');"
    "        div.className = 'cat-radio-item';"
    "        div.onclick = (function(cName) {"
    "          return function() {"
    "            var r = document.getElementById('radio-' + cName);"
    "            if (r) r.checked = true;"
    "            updateDownloadButtonText();"
    "          };"
    "        })(catName);"
    "        var radio = document.createElement('input');"
    "        radio.type = 'radio';"
    "        radio.name = 'current-category';"
    "        radio.id = 'radio-' + catName;"
    "        radio.value = catName;"
    "        if (i === 1) radio.checked = true;"
    "        var label = document.createElement('label');"
    "        label.htmlFor = 'radio-' + catName;"
    "        label.innerHTML = catName + ' <span style=\"font-size:0.75rem;color:var(--text-secondary);float:right;\">(Nächstes Bild: #' + categoryCounters[catName] + ')</span>';"
    "        div.appendChild(radio);"
    "        div.appendChild(label);"
    "        container.appendChild(div);"
    "      }"
    "      projectInitialized = true;"
    "      document.getElementById('categories-section').style.display = 'block';"
    "      showStatus('Projekt \"' + projName + '\" mit ' + n + ' Kategorien initialisiert.', 'success');"
    "      updateDownloadButtonText();"
    "    }"
    "    async function downloadSnapshot() {"
    "      if (!projectInitialized) return;"
    "      var projName = document.getElementById('input-project-name').value.trim().replace(/[^a-zA-Z0-9_-]/g, '') || 'Projekt';"
    "      var activeRadio = document.querySelector('input[name=\"current-category\"]:checked');"
    "      var categoryName = activeRadio ? activeRadio.value : 'cat_1';"
    "      var nextNum = categoryCounters[categoryName];"
    "      var filename = projName + '_' + categoryName + '_Bild_' + nextNum + '.jpg';"
    "      try {"
    "        showStatus('Lade Bild vom ESP32...', '');"
    "        var response = await fetch('/snapshot.jpg');"
    "        if (!response.ok) {"
    "          throw new Error('Server-Fehler beim Laden des Bildes: ' + response.statusText);"
    "        }"
    "        var blob = await response.blob();"
    "        var url = URL.createObjectURL(blob);"
    "        var a = document.createElement('a');"
    "        a.href = url;"
    "        a.download = filename;"
    "        document.body.appendChild(a);"
    "        a.click();"
    "        document.body.removeChild(a);"
    "        URL.revokeObjectURL(url);"
    "        showStatus('Bild erfolgreich heruntergeladen: ' + filename, 'success');"
    "        categoryCounters[categoryName]++;"
    "        var label = document.querySelector('label[for=\"radio-' + categoryName + '\"]');"
    "        if (label) {"
    "          label.innerHTML = categoryName + ' <span style=\"font-size:0.75rem;color:var(--text-secondary);float:right;\">(Nächstes Bild: #' + categoryCounters[categoryName] + ')</span>';"
    "        }"
    "        updateDownloadButtonText();"
    "      } catch (err) {"
    "        console.error(err);"
    "        showStatus('Download-Fehler: ' + err.message, 'error');"
    "      }"
    "    }"
    "    function startStream() {"
    "      stopStream();"
    "      jmuxer = new JMuxer({"
    "        node: player,"
    "        mode: 'video',"
    "        flushingTime: 10,"
    "        fps: 25,"
    "        debug: false"
    "      });"
    "      controller = new AbortController();"
    "      var signal = controller.signal;"
    "      statusDot.style.backgroundColor = '#10b981';"
    "      statusDot.style.boxShadow = '0 0 8px #10b981';"
    "      statusText.textContent = 'Streaming';"
    "      fetch('/stream', { signal: signal })"
    "        .then(response => {"
    "          var reader = response.body.getReader();"
    "          function read() {"
    "            reader.read().then(({ done, value }) => {"
    "              if (done) return;"
    "              if (jmuxer) {"
    "                jmuxer.feed({ video: value });"
    "                if (player.buffered && player.buffered.length > 0) {"
    "                  var bufferEnd = player.buffered.end(player.buffered.length - 1);"
    "                  var delay = bufferEnd - player.currentTime;"
    "                  if (delay > 1.2) {"
    "                    player.currentTime = bufferEnd - 0.1;"
    "                    if (player.playbackRate !== 1.0) player.playbackRate = 1.0;"
    "                  } else if (delay > 0.2) {"
    "                    if (player.playbackRate !== 1.2) player.playbackRate = 1.2;"
    "                  } else if (delay < 0.1) {"
    "                    if (player.playbackRate !== 1.0) player.playbackRate = 1.0;"
    "                  }"
    "                }"
    "              }"
    "              read();"
    "            }).catch(err => console.log('Stream read aborted/error:', err));"
    "          }"
    "          read();"
    "        }).catch(err => console.log('Fetch error:', err));"
    "    }"
    "    function stopStream() {"
    "      if (controller) {"
    "        controller.abort();"
    "        controller = null;"
    "      }"
    "      if (jmuxer) {"
    "        jmuxer.destroy();"
    "        jmuxer = null;"
    "      }"
    "      var faceBox = document.getElementById('face-box');"
    "      if (faceBox) faceBox.style.display = 'none';"
    "      statusDot.style.backgroundColor = '#ef4444';"
    "      statusDot.style.boxShadow = '0 0 8px #ef4444';"
    "      statusText.textContent = 'Stopped';"
    "    }"
    "    function toggleScaling() {"
    "      scaleFactor = (scaleFactor % 3) + 1;"
    "      btnScale.textContent = 'Scale: ' + scaleFactor + 'x';"
    "      updateImageDimensions();"
    "    }"
    "    function formatBytes(bytes) {"
    "      if (bytes === 0) return '0 B';"
    "      const k = 1024;"
    "      const sizes = ['B', 'KB', 'MB'];"
    "      const i = Math.floor(Math.log(bytes) / Math.log(k));"
    "      return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];"
    "    }"
    "    function updateStats() {"
    "      fetch('/status')"
    "        .then(response => response.json())"
    "        .then(data => {"
    "          actualWidth = data.width;"
    "          actualHeight = data.height;"
    "          document.getElementById('val-res').textContent = actualWidth + ' x ' + actualHeight + ' (H.264)';"
    "          updateImageDimensions();"
    "          let sec = data.uptime;"
    "          let hrs = Math.floor(sec / 3600);"
    "          let mins = Math.floor((sec % 3600) / 60);"
    "          let secs = sec % 60;"
    "          document.getElementById('val-uptime').textContent = "
    "            (hrs > 0 ? hrs + 'h ' : '') + mins + 'm ' + secs + 's';"
    "          let dbm = data.rssi;"
    "          let pct = Math.min(Math.max(2 * (dbm + 100), 0), 100);"
    "          document.getElementById('val-wifi').textContent = dbm + ' dBm (' + pct + '%)';"
    "          document.getElementById('val-sram').textContent = formatBytes(data.heap_internal);"
    "          document.getElementById('val-spiram').textContent = formatBytes(data.heap_spiram);"
    "          document.getElementById('val-cpu0').textContent = data.cpu0.toFixed(1) + ' %';"
    "          document.getElementById('val-cpu1').textContent = data.cpu1.toFixed(1) + ' %';"
    "        })"
    "        .catch(err => console.error('Failed to fetch stats', err));"
    "    }"
    "    var valGesture = document.getElementById('val-gesture');"
    "    var valGestureScore = document.getElementById('val-gesture-score');"
    "    var gestureEvtSrc = new EventSource('/gesture-events');"
    "    gestureEvtSrc.onmessage = function(e) {"
    "      var d = JSON.parse(e.data);"
    "      var faceBox = document.getElementById('face-box');"
    "      if (d.gesture === 'no_face') {"
    "        valGesture.textContent = 'no_face';"
    "        valGesture.style.color = 'var(--text-secondary)';"
    "        valGestureScore.textContent = '0.0%';"
    "        if (faceBox) faceBox.style.display = 'none';"
    "      } else {"
    "        valGesture.textContent = d.gesture;"
    "        valGesture.style.color = '#38bdf8';"
    "        valGestureScore.textContent = (d.score * 100).toFixed(1) + '%';"
    "        if (faceBox && d.x1 !== undefined) {"
    "          faceBox.style.left = (d.x1 * scaleFactor) + 'px';"
    "          faceBox.style.top = (d.y1 * scaleFactor) + 'px';"
    "          faceBox.style.width = ((d.x2 - d.x1) * scaleFactor) + 'px';"
    "          faceBox.style.height = ((d.y2 - d.y1) * scaleFactor) + 'px';"
    "          faceBox.style.display = 'block';"
    "        } else if (faceBox) {"
    "          faceBox.style.display = 'none';"
    "        }"
    "      }"
    "    };"
    "    gestureEvtSrc.onerror = function() {"
    "      valGesture.textContent = '...';"
    "      valGesture.style.color = 'var(--text-secondary)';"
    "      valGestureScore.textContent = '-';"
    "    };"
    "    setInterval(updateStats, 1000);"
    "    updateStats();"
    "    startStream();"
    "  </script>"
    "</body>"
    "</html>";

// ---------------------------------------------------------------------------
// Hand Gesture SSE Sender Task & Handlers
// ---------------------------------------------------------------------------
static void sse_sender_task(void *arg)
{
    char cur_gesture[64] = "";
    char event_buf[256];

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));

        char  name[64];
        float score;
        int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
        face_detect_task_get_result(name, sizeof(name), &score, &x1, &y1, &x2, &y2);

        bool is_face = (strcmp(name, "face") == 0);
        bool name_changed = (strcmp(name, cur_gesture) != 0);
        bool should_send = name_changed || is_face;

        if (name_changed) {
            memcpy(cur_gesture, name, sizeof(cur_gesture));
            cur_gesture[sizeof(cur_gesture) - 1] = '\0';
        }

        if (xSemaphoreTake(s_sse_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            continue;
        }

        if (!should_send) {
            xSemaphoreGive(s_sse_mutex);
            continue;
        }

        int len = snprintf(event_buf, sizeof(event_buf),
                           "data:{\"gesture\":\"%s\",\"score\":%.4f,\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d}\n\n",
                           name, score, x1, y1, x2, y2);

        for (int i = 0; i < SSE_MAX_CLIENTS; i++) {
            if (!s_sse_clients[i].req) {
                continue;
            }
            if (httpd_resp_send_chunk(s_sse_clients[i].req, event_buf, len) != ESP_OK) {
                httpd_req_async_handler_complete(s_sse_clients[i].req);
                s_sse_clients[i].req = NULL;
                ESP_LOGI(TAG, "SSE client %d disconnected", i);
            }
        }

        xSemaphoreGive(s_sse_mutex);
    }
}

static esp_err_t sse_handler(httpd_req_t *req)
{
    // Find a free slot.
    int slot = -1;
    if (xSemaphoreTake(s_sse_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        for (int i = 0; i < SSE_MAX_CLIENTS; i++) {
            if (!s_sse_clients[i].req) { slot = i; break; }
        }
        xSemaphoreGive(s_sse_mutex);
    }
    if (slot < 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SSE full");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Accel-Buffering", "no");

    char  init_name[64];
    float init_score;
    int init_x1 = 0, init_y1 = 0, init_x2 = 0, init_y2 = 0;
    face_detect_task_get_result(init_name, sizeof(init_name), &init_score, &init_x1, &init_y1, &init_x2, &init_y2);
    char init_ev[256];
    int  init_len = snprintf(init_ev, sizeof(init_ev),
                             "data:{\"gesture\":\"%s\",\"score\":%.4f,\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d}\n\n",
                             init_name, init_score, init_x1, init_y1, init_x2, init_y2);
    if (httpd_resp_send_chunk(req, init_ev, init_len) != ESP_OK) {
        return ESP_FAIL;
    }

    // Hand the socket to the sender task; handler returns immediately.
    httpd_req_t *async_req;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_sse_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        s_sse_clients[slot].req = async_req;
        xSemaphoreGive(s_sse_mutex);
        ESP_LOGI(TAG, "SSE client %d connected", slot);
    } else {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t gesture_handler(httpd_req_t *req)
{
    char  name[64] = "no_face";
    float score    = 0.0f;
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    face_detect_task_get_result(name, sizeof(name), &score, &x1, &y1, &x2, &y2);

    char response[256];
    int  len = snprintf(response, sizeof(response),
                        "{\"gesture\":\"%s\",\"score\":%.6f,\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d}", name, score, x1, y1, x2, y2);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, response, len);
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Connection", "close"); // Free socket immediately
    return httpd_resp_send(req, STREAM_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

static void get_cpu_usage(float *cpu0_load, float *cpu1_load)
{
    static uint32_t last_idle_time[2] = {0};
    static int64_t last_calc_time = 0;

    int64_t now = esp_timer_get_time();
    int64_t delta_time = now - last_calc_time;

    uint32_t current_idle_time[2] = {0};
    bool found[2] = {false, false};

    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    TaskStatus_t *status_array = malloc(task_count * sizeof(TaskStatus_t));
    if (status_array) {
        task_count = uxTaskGetSystemState(status_array, task_count, NULL);
        for (UBaseType_t i = 0; i < task_count; i++) {
            if (strcmp(status_array[i].pcTaskName, "IDLE0") == 0 || strcmp(status_array[i].pcTaskName, "IDLE") == 0) {
                current_idle_time[0] = status_array[i].ulRunTimeCounter;
                found[0] = true;
            } else if (strcmp(status_array[i].pcTaskName, "IDLE1") == 0) {
                current_idle_time[1] = status_array[i].ulRunTimeCounter;
                found[1] = true;
            }
        }
        free(status_array);
    }

    if (delta_time > 0) {
        if (found[0] && last_calc_time > 0) {
            uint32_t idle_delta = current_idle_time[0] - last_idle_time[0];
            float load = 100.0f - ((float)idle_delta * 100.0f / (float)delta_time);
            if (load < 0) load = 0;
            if (load > 100) load = 100;
            *cpu0_load = load;
        } else {
            *cpu0_load = 0;
        }

        if (found[1] && last_calc_time > 0) {
            uint32_t idle_delta = current_idle_time[1] - last_idle_time[1];
            float load = 100.0f - ((float)idle_delta * 100.0f / (float)delta_time);
            if (load < 0) load = 0;
            if (load > 100) load = 100;
            *cpu1_load = load;
        } else {
            *cpu1_load = 0;
        }
    } else {
        *cpu0_load = 0;
        *cpu1_load = 0;
    }

    if (found[0]) last_idle_time[0] = current_idle_time[0];
    if (found[1]) last_idle_time[1] = current_idle_time[1];
    last_calc_time = now;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    // ESP_LOGI(TAG, "status_handler: called");
    char json_response[256];

    // Get WiFi RSSI
    wifi_ap_record_t ap_info;
    int rssi = -100;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = ap_info.rssi;
    }

    // Get free heap sizes
    uint32_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    // Get uptime
    uint64_t uptime_s = esp_timer_get_time() / 1000000;

    // Get CPU usage
    float cpu0_load = 0;
    float cpu1_load = 0;
    get_cpu_usage(&cpu0_load, &cpu1_load);

    snprintf(json_response, sizeof(json_response),
             "{"
             "\"uptime\":%llu,"
             "\"rssi\":%d,"
             "\"heap_internal\":%lu,"
             "\"heap_spiram\":%lu,"
             "\"width\":%d,"
             "\"height\":%d,"
             "\"cpu0\":%.1f,"
             "\"cpu1\":%.1f"
             "}",
             uptime_s, rssi, (unsigned long)free_internal, (unsigned long)free_spiram,
             FINAL_WIDTH, FINAL_HEIGHT, cpu0_load, cpu1_load);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close"); // Free socket immediately
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    uint32_t size = 0;
    uint8_t *rgb_buf = image_processor_lock_ai_buffer(&size);
    if (!rgb_buf) {
        ESP_LOGE(TAG, "Failed to lock AI RGB888 buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer lock failed");
        return ESP_FAIL;
    }

    jpeg_enc_config_t enc_cfg = {
        .width = FINAL_WIDTH,
        .height = FINAL_HEIGHT,
        .src_type = JPEG_PIXEL_FORMAT_RGB888,
        .subsampling = JPEG_SUBSAMPLE_420,
        .quality = 90,
        .rotate = JPEG_ROTATE_0D,
        .task_enable = false,
    };

    jpeg_enc_handle_t jpeg_enc = NULL;
    jpeg_error_t err = jpeg_enc_open(&enc_cfg, &jpeg_enc);
    if (err != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "Failed to open jpeg encoder: %d", err);
        image_processor_unlock_ai_buffer();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Encoder init failed");
        return ESP_FAIL;
    }

    int outbuf_size = FINAL_WIDTH * FINAL_HEIGHT * 3;
    uint8_t *out_buf = (uint8_t *)heap_caps_malloc(outbuf_size, MALLOC_CAP_SPIRAM);
    if (!out_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG output buffer");
        jpeg_enc_close(jpeg_enc);
        image_processor_unlock_ai_buffer();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    int out_size = 0;
    err = jpeg_enc_process(jpeg_enc, rgb_buf, size, out_buf, outbuf_size, &out_size);

    jpeg_enc_close(jpeg_enc);
    image_processor_unlock_ai_buffer();

    if (err != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "JPEG encoding failed: %d", err);
        free(out_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Encoding failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    esp_err_t res = httpd_resp_send(req, (const char *)out_buf, out_size);
    free(out_buf);
    return res;
}

struct async_resp_arg {
    httpd_req_t *req;
    int client_idx;
};

static void async_stream_task(void *arg)
{
    struct async_resp_arg *resp_arg = (struct async_resp_arg *)arg;
    httpd_req_t *req = resp_arg->req;
    int client_idx = resp_arg->client_idx;
    ESP_LOGI(TAG, "async_stream_task: started for client_idx=%d", client_idx);
    esp_err_t res = ESP_OK;

    uint8_t *local_buf = (uint8_t *)heap_caps_malloc(s_buf_capacity, MALLOC_CAP_SPIRAM);
    if (!local_buf) {
        httpd_resp_send_500(req);
        httpd_req_async_handler_complete(req);
        free(resp_arg);
        vTaskDelete(NULL);
        return;
    }

    while (res == ESP_OK) {
        // Wait for frame ready signal (Timeout 5 seconds)
        if (xSemaphoreTake(s_clients[client_idx].frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            continue;
        }

        size_t local_size = 0;
        if (xSemaphoreTake(s_buf_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            memcpy(local_buf, s_shared_buf, s_shared_size);
            local_size = s_shared_size;
            xSemaphoreGive(s_buf_mutex);
        }
        if (local_size == 0) {
            continue;
        }

        // Send raw H264 chunk
        res = httpd_resp_send_chunk(req, (const char *)local_buf, (ssize_t)local_size);
    }

    // Unregister client
    if (xSemaphoreTake(s_buf_mutex, portMAX_DELAY) == pdTRUE) {
        s_clients[client_idx].active = false;
        if (s_clients[client_idx].frame_ready) {
            vSemaphoreDelete(s_clients[client_idx].frame_ready);
            s_clients[client_idx].frame_ready = NULL;
        }
        xSemaphoreGive(s_buf_mutex);
    }

    ESP_LOGI(TAG, "async_stream_task: finishing for client_idx=%d, res=%d", client_idx, res);
    free(local_buf);
    httpd_resp_send_chunk(req, NULL, 0);
    httpd_req_async_handler_complete(req);
    free(resp_arg);
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    int client_idx = -1;
    ESP_LOGI(TAG, "stream_handler: called");

    // Register client
    if (xSemaphoreTake(s_buf_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
            if (!s_clients[i].active) {
                s_clients[i].frame_ready = xSemaphoreCreateBinary();
                if (s_clients[i].frame_ready != NULL) {
                    s_clients[i].active = true;
                    client_idx = i;
                }
                break;
            }
        }
        xSemaphoreGive(s_buf_mutex);
    }

    if (client_idx < 0) {
        ESP_LOGE(TAG, "Max clients reached, rejecting client");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Max clients reached");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "video/h264");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    // Begin asynchronous request handling to prevent blocking the main HTTPD task loop (which froze `/status`)
    httpd_req_t *async_req = NULL;
    esp_err_t err = httpd_req_async_handler_begin(req, &async_req);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start async handler: %s", esp_err_to_name(err));
        if (xSemaphoreTake(s_buf_mutex, portMAX_DELAY) == pdTRUE) {
            s_clients[client_idx].active = false;
            if (s_clients[client_idx].frame_ready) {
                vSemaphoreDelete(s_clients[client_idx].frame_ready);
                s_clients[client_idx].frame_ready = NULL;
            }
            xSemaphoreGive(s_buf_mutex);
        }
        return err;
    }

    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    if (!resp_arg) {
        ESP_LOGE(TAG, "Failed to allocate async resp arg");
        httpd_req_async_handler_complete(async_req);
        if (xSemaphoreTake(s_buf_mutex, portMAX_DELAY) == pdTRUE) {
            s_clients[client_idx].active = false;
            if (s_clients[client_idx].frame_ready) {
                vSemaphoreDelete(s_clients[client_idx].frame_ready);
                s_clients[client_idx].frame_ready = NULL;
            }
            xSemaphoreGive(s_buf_mutex);
        }
        return ESP_ERR_NO_MEM;
    }
    resp_arg->req = async_req;
    resp_arg->client_idx = client_idx;

    // Delegate stream sending loop to a dedicated thread task pinned to Core 1
    BaseType_t ret = xTaskCreatePinnedToCore(async_stream_task, "async_stream", 4096, resp_arg, 4, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create async stream task");
        free(resp_arg);
        httpd_req_async_handler_complete(async_req);
        if (xSemaphoreTake(s_buf_mutex, portMAX_DELAY) == pdTRUE) {
            s_clients[client_idx].active = false;
            if (s_clients[client_idx].frame_ready) {
                vSemaphoreDelete(s_clients[client_idx].frame_ready);
                s_clients[client_idx].frame_ready = NULL;
            }
            xSemaphoreGive(s_buf_mutex);
        }
        return ESP_FAIL;
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
esp_err_t video_server_init(size_t buf_size)
{
    s_buf_capacity = buf_size;

    s_shared_buf = (uint8_t *)heap_caps_aligned_calloc(64, 1, buf_size, MALLOC_CAP_SPIRAM);
    if (!s_shared_buf) {
        ESP_LOGE(TAG, "Failed to alloc shared video buffer");
        return ESP_ERR_NO_MEM;
    }

    s_buf_mutex = xSemaphoreCreateMutex();
    if (!s_buf_mutex) {
        ESP_LOGE(TAG, "Failed to create buffer mutex");
        return ESP_ERR_NO_MEM;
    }

    // Single HTTP server for everything (Dashboard, API, Stream) on Port 80
    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.server_port      = 80;
    config.ctrl_port        = 32768;
    config.max_open_sockets = MAX_STREAM_CLIENTS + 3;
    config.stack_size       = 8192;
    config.lru_purge_enable = true; // Auto-close oldest socket if full
    config.core_id          = 1;    // Run HTTP server on Core 1 to free Core 0 for AI inference
    config.task_priority    = 4;    // Match priority of AI to prevent starvation

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    httpd_uri_t root_uri = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = root_handler,
    };
    httpd_register_uri_handler(s_server, &root_uri);

    httpd_uri_t status_uri = {
        .uri     = "/status",
        .method  = HTTP_GET,
        .handler = status_handler,
    };
    httpd_register_uri_handler(s_server, &status_uri);

    httpd_uri_t stream_uri = {
        .uri     = "/stream",
        .method  = HTTP_GET,
        .handler = stream_handler,
    };
    httpd_register_uri_handler(s_server, &stream_uri);

    httpd_uri_t snapshot_uri = {
        .uri     = "/snapshot.jpg",
        .method  = HTTP_GET,
        .handler = snapshot_handler,
    };
    httpd_register_uri_handler(s_server, &snapshot_uri);

    httpd_uri_t gesture_uri = {
        .uri     = "/gesture",
        .method  = HTTP_GET,
        .handler = gesture_handler,
    };
    httpd_register_uri_handler(s_server, &gesture_uri);

    httpd_uri_t sse_uri = {
        .uri     = "/gesture-events",
        .method  = HTTP_GET,
        .handler = sse_handler,
    };
    httpd_register_uri_handler(s_server, &sse_uri);

    // SSE mutex and sender task
    s_sse_mutex = xSemaphoreCreateMutex();
    if (!s_sse_mutex) {
        ESP_LOGE(TAG, "Failed to create SSE mutex");
        return ESP_ERR_NO_MEM;
    }
    xTaskCreatePinnedToCore(sse_sender_task, "sse_sender", 4096, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "HTTP server started on port 80 (serving /, /status, /stream, /snapshot.jpg, /gesture, and /gesture-events)");
    return ESP_OK;
}

void video_server_push_frame(const uint8_t *data, size_t size)
{
    if (!s_buf_mutex || !s_shared_buf) {
        return;
    }
    if (size > s_buf_capacity) {
        return;
    }
    if (xSemaphoreTake(s_buf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(s_shared_buf, data, size);
        s_shared_size = size;

        // Signal all active clients
        for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
            if (s_clients[i].active && s_clients[i].frame_ready) {
                xSemaphoreGive(s_clients[i].frame_ready);
            }
        }

        xSemaphoreGive(s_buf_mutex);
    }
}
