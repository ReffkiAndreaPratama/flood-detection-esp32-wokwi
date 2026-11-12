/*
  FloodGuard X — Smart Flood Intelligence System
  Board: ESP32 DevKit C V4 | Wokwi simulation
  Sensors: 2x HC-SR04 (upstream/downstream), 2x potentiometer (rain/flow proxy)
  Outputs: SSD1306 OLED, RGB status LED, buzzer, servo floodgate actuator
  Dashboard: HTTP port 80; Wokwi Private IoT Gateway forwards localhost:8180
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

WebServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Servo floodgate;

// ---------------- Pin map ----------------
constexpr uint8_t PIN_UP_TRIG = 5;
constexpr uint8_t PIN_UP_ECHO = 34;
constexpr uint8_t PIN_DOWN_TRIG = 18;
constexpr uint8_t PIN_DOWN_ECHO = 35;
constexpr uint8_t PIN_RAIN = 32;
constexpr uint8_t PIN_FLOW = 33;
constexpr uint8_t PIN_SERVO = 13;
constexpr uint8_t PIN_BUZZER = 19;
constexpr uint8_t PIN_RGB_R = 25;
constexpr uint8_t PIN_RGB_G = 26;
constexpr uint8_t PIN_RGB_B = 27;
constexpr uint8_t PIN_MUTE = 23;
constexpr uint8_t PIN_MODE = 4;

// ---------------- Model calibration ----------------
constexpr float REFERENCE_HEIGHT_CM = 300.0f;
constexpr unsigned long SAMPLE_INTERVAL_MS = 1000;
constexpr unsigned long TREND_WINDOW_MS = 10000;
constexpr float CRITICAL_LEVEL_CM = 220.0f;

float warningThreshold = 100.0f;
float alertThreshold = 160.0f;
float dangerThreshold = 220.0f;
float rainThreshold = 65.0f;

enum RiskLevel : uint8_t { NORMAL=0, WATCH=1, ALERT=2, DANGER=3 };
enum ControlMode : uint8_t { AUTO_MODE=0, MANUAL_MODE=1 };

float upstreamDistance = 150.0f;
float downstreamDistance = 150.0f;
float upstreamLevel = 150.0f;
float downstreamLevel = 150.0f;
float rainPct = 0.0f;
float flowProxy = 0.0f;
float upstreamRate = 0.0f;       // cm/min
float downstreamRate = 0.0f;     // cm/min
float riskScore = 0.0f;
float etaCriticalMin = -1.0f;
bool sensorsValid = false;
bool alarmMuted = false;
bool manualGateOpen = false;
bool gateOpen = false;
bool testAlarm = false;
RiskLevel risk = NORMAL;
ControlMode mode = AUTO_MODE;
unsigned long lastSample = 0;
unsigned long lastButtonMute = 0;
unsigned long lastButtonMode = 0;
bool prevMute = HIGH;
bool prevMode = HIGH;
unsigned long lastBuzzer = 0;
RiskLevel previousRisk = NORMAL;
String events[10];
uint8_t eventHead = 0;
float oldUpLevel = -1.0f;
float oldDownLevel = -1.0f;
unsigned long oldTrendTime = 0;
unsigned long testAlarmUntil = 0;

const char DASHBOARD[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FloodGuard X | Command Center</title>
<style>
:root{--bg:#07111e;--panel:#101f31;--panel2:#14263b;--line:#253b54;--muted:#91a8c0;--text:#edf6ff;--cyan:#38c8e8;--green:#20c997;--yellow:#f6c94b;--orange:#ff9857;--red:#ff5267}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(ellipse at 15% 0%,#12304a 0,transparent 42%),var(--bg);color:var(--text);font:14px Inter,system-ui,Segoe UI,Arial}
main{max-width:1240px;margin:auto;padding:22px}.head{display:flex;align-items:center;justify-content:space-between;gap:15px;flex-wrap:wrap}
.brand{display:flex;align-items:center;gap:12px}.logo{width:46px;height:46px;border-radius:14px;background:linear-gradient(145deg,#0ea5c6,#155e91);display:grid;place-items:center;font-size:25px;box-shadow:0 8px 24px #079bc544}
h1{font-size:24px;margin:0;letter-spacing:.3px}.sub{color:var(--muted);font-size:12px;margin-top:3px}.live{border:1px solid #23506b;border-radius:99px;padding:8px 13px;color:#8fe8fa;background:#0a2436}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:13px;margin-top:19px}.panel{background:linear-gradient(155deg,#14263b,#0e1b2c);border:1px solid var(--line);border-radius:16px;padding:16px;box-shadow:0 14px 32px #0002}
.kicker{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:1px}.num{font-size:29px;font-weight:800;margin:7px 0 2px}.unit{color:var(--muted);font-size:12px}.bar{height:8px;background:#263b52;border-radius:20px;overflow:hidden;margin-top:13px}.bar span{display:block;height:100%;width:0;background:var(--cyan);transition:width .4s}
.bigstatus{display:flex;align-items:center;gap:10px;margin:12px 0 5px;font-size:21px;font-weight:800}.lamp{width:12px;height:12px;border-radius:50%;background:var(--green);box-shadow:0 0 16px currentColor}
.layout{display:grid;grid-template-columns:1.35fr .85fr;gap:13px;margin-top:13px}.twocol{display:grid;grid-template-columns:1fr 1fr;gap:12px}.panel h2{font-size:14px;margin:0 0 14px;letter-spacing:.2px}
.metric{display:flex;justify-content:space-between;padding:9px 0;border-bottom:1px solid #22364c;color:#c6d6e6}.metric b{color:white}
button{border:0;border-radius:9px;padding:10px 13px;margin:4px 5px 0 0;font-weight:700;color:white;background:#087eaa;cursor:pointer}button.alt{background:#30445b}button.red{background:#a72b42}
.ctrl{margin:14px 0}.ctrl label{display:flex;justify-content:space-between;color:var(--muted);font-size:12px;margin-bottom:7px}input[type=range]{width:100%;accent-color:#28b9dc}
.logs{list-style:none;margin:0;padding:0}.logs li{border-bottom:1px solid #24384e;padding:9px 0;color:#c8d7e7;font-size:12px}
.note{color:var(--muted);font-size:11px;line-height:1.5;margin-top:14px}.tag{display:inline-block;border:1px solid var(--line);border-radius:6px;padding:4px 7px;color:var(--muted);font-size:10px}
.gauge{height:105px;display:flex;align-items:center;justify-content:center;flex-direction:column}.gauge strong{font-size:36px}.gauge small{color:var(--muted)}
@media(max-width:850px){.grid{grid-template-columns:repeat(2,1fr)}.layout{grid-template-columns:1fr}}@media(max-width:480px){main{padding:12px}.grid{gap:8px}.panel{padding:13px}.num{font-size:24px}.twocol{grid-template-columns:1fr}}
</style></head><body><main>
<header class="head"><div class="brand"><div class="logo">🌊</div><div><h1>FLOODGUARD X</h1><div class="sub">SMART FLOOD INTELLIGENCE · ESP32 COMMAND CENTER</div></div></div><div class="live" id="conn">● CONNECTING</div></header>
<section class="grid">
<div class="panel"><div class="kicker">Upstream water level</div><div class="num" id="up">--</div><div class="unit">cm estimated depth</div><div class="bar"><span id="upbar"></span></div></div>
<div class="panel"><div class="kicker">Downstream water level</div><div class="num" id="down">--</div><div class="unit">cm estimated depth</div><div class="bar"><span id="downbar"></span></div></div>
<div class="panel"><div class="kicker">Rain intensity</div><div class="num" id="rain">--</div><div class="unit">simulated %</div><div class="bar"><span id="rainbar"></span></div></div>
<div class="panel"><div class="kicker">Flood risk index</div><div class="gauge"><strong id="riskScore">--</strong><small>RISK SCORE / 100</small></div><div class="bar"><span id="riskbar"></span></div></div>
</section>
<section class="layout">
<div>
<div class="panel"><h2>◉ LIVE SITUATION</h2><div class="bigstatus"><span class="lamp" id="lamp"></span><span id="riskName">--</span></div><div class="unit" id="riskDetail">Waiting for sensor data</div>
<div class="twocol" style="margin-top:15px"><div><div class="kicker">Upstream rise rate</div><div class="num" id="uprate">--</div><div class="unit">cm / min</div></div><div><div class="kicker">Downstream rise rate</div><div class="num" id="downrate">--</div><div class="unit">cm / min</div></div></div>
<div class="metric"><span>Estimated time to critical</span><b id="eta">--</b></div><div class="metric"><span>Flow proxy</span><b id="flow">--</b></div><div class="metric"><span>Gate actuator</span><b id="gate">--</b></div><div class="metric"><span>Operating mode</span><b id="mode">AUTO</b></div>
</div>
<div class="panel" style="margin-top:13px"><h2>⚙ CONTROL CENTER</h2><div class="unit">Choose automatic response or manual gate operation.</div>
<button onclick="cmd('mode','auto')">AUTO</button><button class="alt" onclick="cmd('mode','manual')">MANUAL</button><button onclick="cmd('gate','open')">OPEN GATE</button><button class="alt" onclick="cmd('gate','close')">CLOSE GATE</button><br>
<button class="red" onclick="cmd('alarm','test')">TEST ALARM</button><button class="alt" onclick="cmd('mute','toggle')">TOGGLE MUTE</button>
<div class="ctrl"><label>Warning level <b id="wv">100 cm</b></label><input id="warning" type="range" min="40" max="180" value="100" oninput="txt('wv',this.value+' cm')" onchange="saveThresholds()"></div>
<div class="ctrl"><label>Alert level <b id="av">160 cm</b></label><input id="alert" type="range" min="80" max="240" value="160" oninput="txt('av',this.value+' cm')" onchange="saveThresholds()"></div>
<div class="ctrl"><label>Danger level <b id="dv">220 cm</b></label><input id="danger" type="range" min="120" max="290" value="220" oninput="txt('dv',this.value+' cm')" onchange="saveThresholds()"></div>
</div></div>
<div>
<div class="panel"><h2>▤ SENSOR TELEMETRY</h2><div class="metric"><span>Upstream distance</span><b id="ud">-- cm</b></div><div class="metric"><span>Downstream distance</span><b id="dd">-- cm</b></div><div class="metric"><span>Rain threshold</span><b id="rt">65%</b></div><div class="metric"><span>Sensor health</span><b id="health">--</b></div><div class="metric"><span>Alarm state</span><b id="muted">--</b></div><div class="note">Trend estimates use a rolling sample comparison. It is a demonstration indicator, not a hydrological forecast.</div></div>
<div class="panel" style="margin-top:13px"><h2>◷ EVENT TIMELINE</h2><ul class="logs" id="logs"><li>No events yet</li></ul><div class="note">Most recent state transitions appear first.</div></div>
</div></section>
<div class="note">FloodGuard X simulation prototype · Not a certified flood warning or life-safety system. Calibrate sensors and thresholds before any field use.</div>
</main><script>
const $=x=>document.getElementById(x),colors=['#20c997','#f6c94b','#ff9857','#ff5267'];
function txt(id,v){$(id).textContent=v}
async function cmd(k,v){try{await fetch('/api/control?'+k+'='+v);await refresh()}catch(e){}}
async function saveThresholds(){let q=new URLSearchParams({warning:$('warning').value,alert:$('alert').value,danger:$('danger').value});await fetch('/api/thresholds?'+q)}
async function refresh(){try{let r=await fetch('/api/status'),d=await r.json();$('conn').textContent='● SYSTEM ONLINE';$('up').textContent=d.up.toFixed(1);$('down').textContent=d.down.toFixed(1);$('rain').textContent=d.rain.toFixed(0);$('riskScore').textContent=d.score.toFixed(0);$('upbar').style.width=Math.min(100,d.up/300*100)+'%';$('downbar').style.width=Math.min(100,d.down/300*100)+'%';$('rainbar').style.width=d.rain+'%';$('riskbar').style.width=d.score+'%';$('riskbar').style.background=colors[d.risk];$('upbar').style.background=colors[d.risk];$('downbar').style.background=colors[d.risk];$('lamp').style.background=colors[d.risk];$('lamp').style.color=colors[d.risk];$('riskName').textContent=d.name;$('riskDetail').textContent=d.detail;$('uprate').textContent=d.uprate.toFixed(1);$('downrate').textContent=d.downrate.toFixed(1);$('eta').textContent=d.eta<0?'Not available':d.eta.toFixed(1)+' min';$('flow').textContent=d.flow.toFixed(0)+'%';$('gate').textContent=d.gate?'OPEN':'CLOSED';$('mode').textContent=d.mode;$('ud').textContent=d.ud.toFixed(1)+' cm';$('dd').textContent=d.dd.toFixed(1)+' cm';$('health').textContent=d.valid?'ONLINE':'CHECK SENSOR';$('muted').textContent=d.muted?'MUTED':'ACTIVE';$('rt').textContent=d.rainth+'%';$('logs').innerHTML=d.events.map(x=>'<li>'+x+'</li>').join('')||'<li>No events yet</li>';
[['warning','wv','warning'],['alert','av','alert'],['danger','dv','danger']].forEach(a=>{if(document.activeElement.id!==a[0]){$(a[0]).value=d[a[2]];$(a[1]).textContent=d[a[2]]+' cm'}})
}catch(e){$('conn').textContent='● CONNECTION LOST'}}
setInterval(refresh,1000);refresh();
</script></body></html>
)HTML";

String riskName(RiskLevel r) {
  switch (r) {
    case WATCH: return "WASPADA";
    case ALERT: return "SIAGA";
    case DANGER: return "BAHAYA";
    default: return "NORMAL";
  }
}

void logEvent(const String& message) {
  events[eventHead] = message;
  eventHead = (eventHead + 1) % 10;
  Serial.println("[EVENT] " + message);
}

float readUltrasonic(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long duration = pulseIn(echo, HIGH, 30000UL);
  if (!duration) return -1.0f;
  return duration * 0.0343f / 2.0f;
}

float toWaterLevel(float distance) {
  return constrain(REFERENCE_HEIGHT_CM - distance, 0.0f, REFERENCE_HEIGHT_CM);
}

void updateTrend(float up, float down, unsigned long now) {
  if (oldTrendTime > 0 && now > oldTrendTime) {
    float minutes = (now - oldTrendTime) / 60000.0f;
    if (minutes > 0.001f) {
      upstreamRate = constrain((up - oldUpLevel) / minutes, -300.0f, 300.0f);
      downstreamRate = constrain((down - oldDownLevel) / minutes, -300.0f, 300.0f);
    }
  }
  oldUpLevel = up; oldDownLevel = down; oldTrendTime = now;
}

void calculateRisk() {
  float maxLevel = max(upstreamLevel, downstreamLevel);
  float trendBoost = max(0.0f, max(upstreamRate, downstreamRate)) * 0.12f;
  float levelScore = constrain(maxLevel / dangerThreshold * 75.0f, 0.0f, 75.0f);
  float rainScore = rainPct * 0.18f;
  float flowScore = flowProxy * 0.08f;
  riskScore = constrain(levelScore + rainScore + flowScore + trendBoost, 0.0f, 100.0f);

  RiskLevel next = NORMAL;
  if (maxLevel >= warningThreshold || rainPct >= rainThreshold) next = WATCH;
  if (maxLevel >= alertThreshold || (rainPct >= rainThreshold && maxLevel >= warningThreshold * 0.85f)) next = ALERT;
  if (maxLevel >= dangerThreshold || riskScore >= 88.0f) next = DANGER;
  risk = next;

  etaCriticalMin = -1.0f;
  float risingRate = max(upstreamRate, downstreamRate);
  if (risingRate > 0.1f && maxLevel < CRITICAL_LEVEL_CM) {
    etaCriticalMin = (CRITICAL_LEVEL_CM - maxLevel) / risingRate;
  }
}

void setRgb(bool red, bool green, bool blue) {
  digitalWrite(PIN_RGB_R, red ? HIGH : LOW);
  digitalWrite(PIN_RGB_G, green ? HIGH : LOW);
  digitalWrite(PIN_RGB_B, blue ? HIGH : LOW);
}

void applyOutputs() {
  switch (risk) {
    case WATCH: setRgb(true, true, false); break;       // yellow
    case ALERT: setRgb(true, true, false); break;      // amber/orange approximation
    case DANGER: setRgb(true, false, false); break;
    default: setRgb(false, true, false); break;
  }

  if (mode == AUTO_MODE) {
    // Demo actuator action: open the floodgate servo when danger is detected.
    gateOpen = (risk == DANGER);
  } else {
    gateOpen = manualGateOpen;
  }
  floodgate.write(gateOpen ? 90 : 0);

  if (alarmMuted || risk == NORMAL) {
    noTone(PIN_BUZZER);
    return;
  }
  unsigned long phase = millis() % 1800UL;
  if (risk == WATCH) {
    if (phase < 150) tone(PIN_BUZZER, 1000); else noTone(PIN_BUZZER);
  } else if (risk == ALERT) {
    if (phase < 180 || (phase > 350 && phase < 530)) tone(PIN_BUZZER, 1500); else noTone(PIN_BUZZER);
  } else {
    if (phase < 420 || (phase > 600 && phase < 1050) || (phase > 1250 && phase < 1650)) tone(PIN_BUZZER, 2100); else noTone(PIN_BUZZER);
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("FLOODGUARD X  " + riskName(risk));
  display.drawLine(0, 11, 127, 11, SSD1306_WHITE);
  display.setCursor(0, 16); display.print("UP: "); display.print(upstreamLevel, 0); display.print("cm");
  display.setCursor(68, 16); display.print("DN: "); display.print(downstreamLevel, 0);
  display.setCursor(0, 29); display.print("RAIN: "); display.print(rainPct, 0); display.print("%");
  display.setCursor(68, 29); display.print("RISK: "); display.print(riskScore, 0);
  display.setCursor(0, 42); display.print("GATE: "); display.print(gateOpen ? "OPEN" : "CLOSED");
  display.setCursor(68, 42); display.print(mode == AUTO_MODE ? "AUTO" : "MANUAL");
  display.setCursor(0, 55); display.print("FLOW: "); display.print(flowProxy, 0); display.print("%");
  display.display();
}

String jsonEscape(const String& s) {
  String out;
  for (size_t i=0;i<s.length();i++) {
    char c=s[i]; if(c=='"' || c=='\\') out+='\\'; if(c>=32) out+=c;
  }
  return out;
}

String statusJson() {
  String ev="[";
  bool first=true;
  for (uint8_t i=0;i<10;i++) {
    uint8_t idx=(eventHead+9-i)%10;
    if(events[idx].length()) { if(!first) ev+=","; ev+="\""+jsonEscape(events[idx])+"\""; first=false; }
  }
  ev+="]";
  String j="{";
  j+="\"up\":"+String(upstreamLevel,1)+",\"down\":"+String(downstreamLevel,1)+",";
  j+="\"ud\":"+String(upstreamDistance,1)+",\"dd\":"+String(downstreamDistance,1)+",";
  j+="\"rain\":"+String(rainPct,1)+",\"flow\":"+String(flowProxy,1)+",";
  j+="\"uprate\":"+String(upstreamRate,2)+",\"downrate\":"+String(downstreamRate,2)+",";
  j+="\"score\":"+String(riskScore,1)+",\"eta\":"+String(etaCriticalMin,1)+",";
  j+="\"risk\":"+String((int)risk)+",\"name\":\""+riskName(risk)+"\",";
  j+="\"detail\":\""+String(risk==DANGER?"Critical water level: automatic actuator response active":risk==ALERT?"High water level: prepare emergency response":risk==WATCH?"Elevated water/rain: monitor closely":"Conditions currently within demo safe range")+"\",";
  j+="\"gate\":"+String(gateOpen?"true":"false")+",\"mode\":\""+String(mode==AUTO_MODE?"AUTO":"MANUAL")+"\",";
  j+="\"valid\":"+String(sensorsValid?"true":"false")+",\"muted\":"+String(alarmMuted?"true":"false")+",";
  j+="\"warning\":"+String(warningThreshold,0)+",\"alert\":"+String(alertThreshold,0)+",\"danger\":"+String(dangerThreshold,0)+",";
  j+="\"rainth\":"+String(rainThreshold,0)+",\"events\":"+ev+"}";
  return j;
}

void handleRoot(){ server.send_P(200,"text/html",DASHBOARD); }
void handleStatus(){ server.send(200,"application/json",statusJson()); }

void handleControl() {
  if(server.hasArg("mode")) {
    String v=server.arg("mode");
    if(v=="auto") mode=AUTO_MODE;
    if(v=="manual") mode=MANUAL_MODE;
    logEvent("Mode changed to "+String(mode==AUTO_MODE?"AUTO":"MANUAL"));
  }
  if(server.hasArg("gate")) {
    String v=server.arg("gate");
    if(v=="open") { mode=MANUAL_MODE; manualGateOpen=true; }
    if(v=="close") { mode=MANUAL_MODE; manualGateOpen=false; }
    logEvent(String("Manual gate command: ")+(manualGateOpen?"OPEN":"CLOSE"));
  }
  if(server.hasArg("mute")) {
    alarmMuted=!alarmMuted;
    logEvent(alarmMuted?"Alarm muted":"Alarm unmuted");
  }
  if(server.hasArg("alarm") && server.arg("alarm")=="test") {
    testAlarmUntil=millis()+500;
    tone(PIN_BUZZER,1900,500);
    logEvent("Alarm test activated");
  }
  server.send(200,"application/json","{\"ok\":true}");
}

void handleThresholds() {
  float w=server.hasArg("warning")?server.arg("warning").toFloat():warningThreshold;
  float a=server.hasArg("alert")?server.arg("alert").toFloat():alertThreshold;
  float d=server.hasArg("danger")?server.arg("danger").toFloat():dangerThreshold;
  w=constrain(w,40.0f,180.0f); a=constrain(a,80.0f,240.0f); d=constrain(d,120.0f,290.0f);
  if(a<=w) a=w+10; if(d<=a) d=a+10;
  warningThreshold=w; alertThreshold=a; dangerThreshold=min(d,299.0f);
  server.send(200,"application/json","{\"ok\":true}");
}

void handleButtons() {
  unsigned long now=millis();
  bool m=digitalRead(PIN_MUTE);
  if(prevMute==HIGH && m==LOW && now-lastButtonMute>220) {
    alarmMuted=!alarmMuted; logEvent(alarmMuted?"Physical mute enabled":"Physical mute disabled"); lastButtonMute=now;
  }
  prevMute=m;
  bool md=digitalRead(PIN_MODE);
  if(prevMode==HIGH && md==LOW && now-lastButtonMode>220) {
    mode=(mode==AUTO_MODE)?MANUAL_MODE:AUTO_MODE;
    if(mode==MANUAL_MODE) manualGateOpen=false;
    logEvent(String("Physical mode switch: ")+(mode==AUTO_MODE?"AUTO":"MANUAL"));
    lastButtonMode=now;
  }
  prevMode=md;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_UP_TRIG,OUTPUT); pinMode(PIN_UP_ECHO,INPUT);
  pinMode(PIN_DOWN_TRIG,OUTPUT); pinMode(PIN_DOWN_ECHO,INPUT);
  pinMode(PIN_RAIN,INPUT); pinMode(PIN_FLOW,INPUT);
  pinMode(PIN_BUZZER,OUTPUT);
  pinMode(PIN_RGB_R,OUTPUT); pinMode(PIN_RGB_G,OUTPUT); pinMode(PIN_RGB_B,OUTPUT);
  pinMode(PIN_MUTE,INPUT_PULLUP); pinMode(PIN_MODE,INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAIN,ADC_11db);
  analogSetPinAttenuation(PIN_FLOW,ADC_11db);

  Wire.begin(21,22);
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)) Serial.println("OLED init failed");
  else { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.println("FLOODGUARD X"); display.println("Booting system..."); display.display(); }

  floodgate.setPeriodHertz(50);
  floodgate.attach(PIN_SERVO,500,2400);
  floodgate.write(0);
  logEvent("FloodGuard X booted");

  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.print("Connecting to Wokwi-GUEST");
  unsigned long start=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-start<15000){delay(300);Serial.print(".");}
  Serial.println();
  if(WiFi.status()==WL_CONNECTED){Serial.print("Dashboard IP: ");Serial.println(WiFi.localIP());}
  else Serial.println("WiFi unavailable; local monitoring continues.");
  server.on("/",HTTP_GET,handleRoot);
  server.on("/api/status",HTTP_GET,handleStatus);
  server.on("/api/control",HTTP_GET,handleControl);
  server.on("/api/thresholds",HTTP_GET,handleThresholds);
  server.begin();
  Serial.println("HTTP server started on port 80");
}

void loop() {
  server.handleClient();
  handleButtons();
  unsigned long now=millis();
  if(now-lastSample>=SAMPLE_INTERVAL_MS) {
    lastSample=now;
    float du=readUltrasonic(PIN_UP_TRIG,PIN_UP_ECHO);
    delay(35); // reduce ultrasonic cross-talk
    float dd=readUltrasonic(PIN_DOWN_TRIG,PIN_DOWN_ECHO);
    sensorsValid=(du>1.5f && du<=REFERENCE_HEIGHT_CM && dd>1.5f && dd<=REFERENCE_HEIGHT_CM);
    if(du>1.5f && du<=REFERENCE_HEIGHT_CM){upstreamDistance=du;upstreamLevel=toWaterLevel(du);}
    if(dd>1.5f && dd<=REFERENCE_HEIGHT_CM){downstreamDistance=dd;downstreamLevel=toWaterLevel(dd);}
    rainPct=analogRead(PIN_RAIN)/4095.0f*100.0f;
    flowProxy=analogRead(PIN_FLOW)/4095.0f*100.0f;
    updateTrend(upstreamLevel,downstreamLevel,now);
    previousRisk=risk;
    calculateRisk();
    if(risk!=previousRisk) logEvent(riskName(previousRisk)+" -> "+riskName(risk)+" | score "+String(riskScore,0));
    applyOutputs();
    updateDisplay();
    Serial.printf("UP %.1f cm | DOWN %.1f cm | RAIN %.0f%% | FLOW %.0f%% | RISK %.0f | %s\n",
      upstreamLevel,downstreamLevel,rainPct,flowProxy,riskScore,riskName(risk).c_str());
  }
}
