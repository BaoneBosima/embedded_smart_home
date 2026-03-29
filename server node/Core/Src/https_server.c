#include "lwip/api.h"
#include "lwip/err.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
// --- Import the shared variables and mutex from freertos.c ---
extern osMutexId_t dashDataMutexHandle;
extern volatile float dash_temp;
extern volatile float dash_press;
extern volatile float dash_hum;
extern volatile uint8_t dash_door_open;
extern volatile uint8_t dash_pir_motion;
extern volatile uint8_t dash_armed;
extern volatile uint8_t dash_alarm;
extern volatile float dash_target_temp;


extern void send_arm_command(uint8_t cmd);
extern void send_target_temp(float temp);
void serve_html_page(struct netconn *conn);

/* Browsers often split POST: headers in the first TCP segment, body in the next.
 * A single netconn_recv() then misses action=arm / action=disarm (short body after long headers). */
#define HTTP_MAX_REQUEST 2048U
static char s_http_rxbuf[HTTP_MAX_REQUEST];

static int http_parse_content_length(const char *req)
{
    const char *cl = strstr(req, "Content-Length:");
    if (cl == NULL) {
        cl = strstr(req, "content-length:");
    }
    if (cl == NULL) {
        return 0;
    }
    cl = strchr(cl, ':');
    if (cl == NULL) {
        return 0;
    }
    cl++;
    while (*cl == ' ' || *cl == '\t') {
        cl++;
    }
    return atoi(cl);
}

/* Receive full HTTP request (headers + Content-Length body) into buf. */
static err_t recv_full_http_request(struct netconn *newconn, char *buf, size_t buf_sz, size_t *out_len)
{
    size_t total = 0;

    *out_len = 0;
    buf[0] = '\0';

    for (;;) {
        struct netbuf *inbuf = NULL;
        if (netconn_recv(newconn, &inbuf) != ERR_OK) {
            return ERR_CLSD;
        }

        char *data;
        u16_t len;
        netbuf_data(inbuf, (void **)&data, &len);
        if (total + (size_t)len >= buf_sz - 1U) {
            netbuf_delete(inbuf);
            return ERR_MEM;
        }
        memcpy(buf + total, data, len);
        total += (size_t)len;
        buf[total] = '\0';
        netbuf_delete(inbuf);

        char *sep = strstr(buf, "\r\n\r\n");
        if (sep != NULL) {
            size_t hdr_end = (size_t)(sep - buf) + 4U;
            int body_len = http_parse_content_length(buf);
            if (body_len < 0) {
                body_len = 0;
            }
            if (total >= hdr_end + (size_t)body_len) {
                *out_len = total;
                return ERR_OK;
            }
        }

        if (total >= buf_sz - 64U) {
            *out_len = total;
            return ERR_OK;
        }
    }
}
/* ------------------------------------------------------------------------ */
/* SERVE HTML PAGE         */
/* ------------------------------------------------------------------------ */
void serve_html_page(struct netconn *conn) {

    static const char html_page[] =
    // =====================================================================
    // 1. HTTP HEADERS: The invisible handshake with the browser
    // =====================================================================
    // "200 OK" tells the browser the connection was successful
    "HTTP/1.1 200 OK\r\n"
    // Tells the browser to interpret the incoming text as a webpage, not a file download.
    // The "\r\n\r\n" is mandatory: it marks the end of the headers and the start of the webpage.
    "Content-Type: text/html\r\n\r\n"

    // =====================================================================
    // 2. HTML SETUP & CSS STYLING
    // =====================================================================
    // Define this as a modern HTML5 document and make it scale nicely on mobile phones
    "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Home Control</title>"

    // Start of the CSS style block
    "<style>"
    // :root defines variables. This makes it easy to change the color scheme later.
    ":root{--bg:#0f172a;--card:#1e293b;--text:#f8fafc;--dim:#94a3b8;--blue:#3b82f6;--red:#ef4444;--green:#22c55e;}"

    // Body: Uses the native system font (looks like Apple on Mac, Windows UI on PC).
    // flex and justify-content center the whole dashboard on wide desktop screens.
    "body{font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--text);margin:0;padding:20px;display:flex;justify-content:center;}"

    // Limits the dashboard width to 800px so it doesn't stretch too far on big monitors
    ".wrap{max-width:800px;width:100%;}"

    // Header styling: Flexbox pushes the title to the left and the status dot to the right
    "h1{font-weight:600;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #334155;padding-bottom:15px;}"

    // The glowing status dot (uses box-shadow to create the glow effect)
    ".dot{height:12px;width:12px;background:var(--green);border-radius:50%;box-shadow:0 0 10px var(--green);}"

    // The CSS Grid automatically creates columns. 'auto-fit' means it will drop to 1 column on phones and 3 on desktops.
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;margin-bottom:24px;}"

    // The base styling for all the dark gray rectangles (cards)
    ".card{background:var(--card);border:1px solid #334155;border-radius:16px;padding:20px;transition:0.3s;}"

    // A special CSS class we trigger via JavaScript to make a card glow red during an alarm
    ".card.alert{border-color:var(--red);box-shadow:0 0 15px rgba(239,68,68,0.2);}"

    // Typography classes for making the numbers huge and the labels small
    ".label{color:var(--dim);font-size:0.85rem;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px;}"
    ".val{font-size:2.5rem;font-weight:700;}"
    ".unit{font-size:1.2rem;color:var(--dim);font-weight:400;}"

    // Button styling. 'transition' makes the hover effect smooth.
    "button{width:100%;padding:14px;border:none;border-radius:10px;font-size:1rem;font-weight:600;cursor:pointer;transition:0.2s;margin-top:10px;}"
    ".btn-red{background:var(--red);color:#fff;}"
    ".btn-gray{background:#334155;color:#fff;}"
    ".btn-blue{background:var(--blue);color:#fff;}"
    "button:hover{filter:brightness(1.2);}" // Makes buttons slightly brighter when hovered

    // Input box styling for the target temperature
    ".flex{display:flex;gap:12px;}"
    "input{width:100%;padding:12px;background:#0f172a;border:1px solid #334155;color:#fff;border-radius:10px;font-size:1.1rem;}"
    "</style></head>"

    // =====================================================================
    // 3. HTML STRUCTURE (The visible parts of the page)
    // =====================================================================
    "<body><div class='wrap'>"
    "<h1>Environment <div class='dot' id='conn'></div></h1>"

    // --- Top Section: Sensor Grid ---
    // Notice how every value has a unique 'id' (like id='temp'). This is how the JavaScript finds exactly where to put the new numbers later.
    "<div class='grid'>"
    "<div class='card'><div class='label'>Temperature</div><div class='val' id='temp'>--<span class='unit'>&deg;C</span></div></div>"
    "<div class='card'><div class='label'>Humidity</div><div class='val' id='hum'>--<span class='unit'>%</span></div></div>"
    "<div class='card'><div class='label'>Pressure</div><div class='val' id='press'>--<span class='unit'>hPa</span></div></div>"
    "<div class='card' id='door-card'><div class='label'>Door Sensor</div><div class='val' id='door'>--</div></div>"
    "<div class='card' id='motion-card'><div class='label'>Motion</div><div class='val' id='motion'>--</div></div>"
    "</div>"

    // --- Middle Section: Security Panel ---
    "<h1>Security Control</h1>"
    "<div class='card' id='sec-card' style='margin-bottom:24px;'>"
    "<div class='flex' style='justify-content:space-between;align-items:center;'>"
    "<div><div class='label'>System Status</div><div class='val' style='font-size:1.8rem;' id='armed'>--</div></div>"
    "<div><div class='label'>Alarm</div><div class='val' style='font-size:1.8rem;' id='alarm'>--</div></div>"
    "</div>"
    // The onclick='' attributes tell the browser to run our custom JavaScript when clicked
    "<div class='flex' style='margin-top:20px;'>"
    "<button class='btn-red' onclick='sendCmd(\"action=arm\")'>Arm System</button>"
    "<button class='btn-gray' onclick='sendCmd(\"action=disarm\")'>Disarm</button>"
    "</div></div>"

    // --- Bottom Section: Climate Panel ---
    "<h1>Climate Control</h1>"
    "<div class='card'>"
    "<div class='label'>Target Temperature: <span id='target' style='color:#fff;font-weight:bold;font-size:1.2rem;'>--</span> &deg;C</div>"
    "<div class='flex' style='margin-top:15px;'>"
    "<input type='number' id='tempIn' value='22' step='0.5'>"
    "<button class='btn-blue' style='margin-top:0;' onclick='setTemp()'>Apply Setpoint</button>"
    "</div></div>"
    "</div>" // Ends the main .wrap div

    // =====================================================================
    // 4. JAVASCRIPT (The background worker that fetches live data)
    // =====================================================================
    "<script>"

    // This is the main function that fetches new data from the board
    "async function update(){"
    "  try {"
    // Reach out to the C-server (serve_json_status) and wait for the JSON reply
    "    let r = await fetch('/api/status');"
    "    let d = await r.json();" // Convert the text reply into a usable JavaScript object

    // Grab the numbers from the JSON and update the HTML text directly.
    // toFixed(1) forces the floats to have exactly 1 decimal place (e.g., 22.0)
    "    document.getElementById('temp').innerHTML = d.temp.toFixed(1) + '<span class=\"unit\">&deg;C</span>';"
    "    document.getElementById('hum').innerHTML = d.humidity.toFixed(1) + '<span class=\"unit\">%</span>';"
    "    document.getElementById('press').innerHTML = d.pressure.toFixed(1) + '<span class=\"unit\">hPa</span>';"

    // For booleans (1 or 0), we use a ternary operator: condition ? "If True" : "If False"
    "    document.getElementById('door').innerText = d.door;"
    "    document.getElementById('motion').innerText = d.motion ? 'DETECTED' : 'Clear';"
    "    document.getElementById('armed').innerText = d.armed ? 'ARMED' : 'Disarmed';"
    "    document.getElementById('alarm').innerText = d.alarm ? 'SOUNDING' : 'Silent';"
    "    document.getElementById('target').innerText = d.target_temp.toFixed(1);"

    // If we successfully received data, make sure the connection dot is green
    "    document.getElementById('conn').style.background = 'var(--green)';"

    // Dynamic Alerts: If the door is open, change the card's CSS class to 'card alert' to make it glow red.
    "    document.getElementById('door-card').className = d.door === 'OPEN' ? 'card alert' : 'card';"
    "    document.getElementById('motion-card').className = d.motion ? 'card alert' : 'card';"
    "    document.getElementById('sec-card').className = d.alarm ? 'card alert' : 'card';"

    // If the network crashes or the board unplugs, this catch block runs and turns the dot red.
    "  } catch(e) { document.getElementById('conn').style.background = 'var(--red)'; }"
    "}"

    // This function handles the ARM and DISARM button clicks. It sends a POST request to handle_command in C.
    "async function sendCmd(b){"
    "  await fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:b});"
    "  update();" // Immediately refresh the data to see if the command worked
    "}"

    // This function handles the Apply Setpoint button. It grabs the number from the input box and sends it.
    "function setTemp(){"
    "  let v = document.getElementById('tempIn').value;"
    "  sendCmd('target_temp=' + v);"
    "}"

    // Setup a timer to automatically run the update() function every 2000 milliseconds (2 seconds)
    "setInterval(update, 2000);"

    // Run the update() function once immediately as soon as the page loads
    "update();"

    "</script></body></html>";

    // Finally, tell LwIP to send this entire massive string out over the Ethernet port
    netconn_write(conn, html_page, strlen(html_page), NETCONN_COPY);
}

/* ------------------------------------------------------------------------ */
/* JSON STATUS ENDPOINT: Reads shared variables safely and sends JSON       */
/* ------------------------------------------------------------------------ */
static void serve_json_status(struct netconn *conn) {
    char response[512];
    float temp, press, hum, target;
    uint8_t door, pir, armed, alarm;

    /* Copy shared state under mutex (copy-and-release pattern) */
    osMutexAcquire(dashDataMutexHandle, osWaitForever);
    temp = dash_temp;
    press = dash_press;
    hum = dash_hum;
    door = dash_door_open;
    pir = dash_pir_motion;
    armed = dash_armed;
    alarm = dash_alarm;
    target = dash_target_temp;
    osMutexRelease(dashDataMutexHandle);

    /* Build JSON response */
    int len = snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n"
        "{\"temp\":%.1f, \"pressure\":%.1f, \"humidity\":%.1f,"
        "\"door\":\"%s\",\"motion\":%s,\"armed\":%s,"
        "\"alarm\":%s, \"target_temp\":%.1f}",
        temp, press, hum,
        door ? "OPEN" : "CLOSED",
        pir ? "true" : "false",
        armed ? "true" : "false",
        alarm ? "true" : "false",
        target);

    netconn_write(conn, response, len, NETCONN_COPY);
}

/* ------------------------------------------------------------------------ */
/* POST COMMAND HANDLER: Parses button clicks from the web page             */
/* ------------------------------------------------------------------------ */
static void handle_command(struct netconn *conn, char *buf, u16_t buflen) {
    /* Find the POST body (after the blank line "\r\n\r\n") */
    char *body = strstr(buf, "\r\n\r\n");

    if (body) {
        body += 4; /* skip past the blank line */

        if (strstr(body, "action=arm")) {
            send_arm_command(0x01); /* CAN TX: 0x200, byte 0x01 */
        }
        else if (strstr(body, "action=disarm")) {
            send_arm_command(0x02); /* CAN TX: 0x200, byte 0x02 */
        }
        else if (strstr(body, "target_temp=")) {
            char *val = strstr(body, "target_temp=") + 12;
            float new_temp = strtof(val, NULL);
            send_target_temp(new_temp); /* CAN TX: 0x201 */
        }
    }

    /* Respond with success JSON */
    const char *resp = "HTTP/1.1 200 OK\r\n"
                       "Content-Type: application/json\r\n\r\n"
                       "{\"status\":\"ok\"}";
    netconn_write(conn, resp, strlen(resp), NETCONN_COPY);
}

/* ------------------------------------------------------------------------ */
/* MAIN SERVER LOOP: The task that sleeps until a browser connects          */
/* ------------------------------------------------------------------------ */
void http_server_init(void) {
    struct netconn *conn, *newconn;
    char *buf;
    size_t buflen;

    /* Create a TCP connection object */
    conn = netconn_new(NETCONN_TCP);

    /* Bind to port 80 (HTTP) on all network interfaces */
    netconn_bind(conn, IP_ADDR_ANY, 80);

    /* Start listening for incoming connections */
    netconn_listen(conn);

    for (;;) {
        /* Block until a browser connects (zero CPU while waiting) */
        if (netconn_accept(conn, &newconn) == ERR_OK) {

            if (recv_full_http_request(newconn, s_http_rxbuf, sizeof(s_http_rxbuf), &buflen) == ERR_OK) {
                buf = s_http_rxbuf;

                /* Route the request based on the first line */
                if (strncmp(buf, "GET / ", 6) == 0) {
                    serve_html_page(newconn);
                }
                else if (strncmp(buf, "GET /api/status", 15) == 0) {
                    serve_json_status(newconn);
                }
                else if (strncmp(buf, "POST /api/command", 17) == 0) {
                    handle_command(newconn, buf, (u16_t)buflen);
                }
            }

            netconn_close(newconn);
            netconn_delete(newconn);
        }
    }
}
