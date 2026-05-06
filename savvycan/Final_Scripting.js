let tickMs = 200;
let tickCount = 0;
let elapsedMs = 0;

let sa = 0x00;

// =====================
// Configuracao do ensaio final
// =====================
// O primeiro evento comeca somente depois de 5 min de operacao normal,
// para que o datalogger consiga preencher a janela PRE.
// A falha simulada dura 5 min, permitindo gerar a janela ACTIVE.
// Depois da falha, o ESP32 mantera o registro por 5 min na janela POST.
const PRE_TEST_MS = 5 * 60 * 1000;
const FAULT_DURATION_MS = 5 * 60 * 1000;
const MIN_NORMAL_BETWEEN_FAULTS_MS = 5 * 60 * 1000;
const EXTRA_NORMAL_MIN_MS = 30 * 1000;
const EXTRA_NORMAL_MAX_MS = 90 * 1000;

// baseline "normal"
let baseRpm = 3000;
let baseDrv = 35;
let baseAct = 30;

let faultActive = false;
let currentFault = null;
let faultEndMs = 0;
let nextFaultAtMs = PRE_TEST_MS;

function setup() {
    host.log("Script J1939 iniciado - ensaio 5 min PRE / 5 min ACTIVE / 5 min POST");
    host.setTickInterval(tickMs);
}

function randInt(min, max) {
    return Math.floor(Math.random() * (max - min + 1)) + min;
}

function clamp(v, min, max) {
    return Math.max(min, Math.min(max, v));
}

function rpmToRaw(rpm) {
    let raw = Math.round(rpm / 0.125);
    raw = clamp(raw, 0, 0xFFFF);
    return raw;
}

// mapeamento de bancada
// raw = valor + 125
function torquePctToRaw(pct) {
    let raw = Math.round(pct + 125);
    raw = clamp(raw, 0, 250);
    return raw;
}

function sendEEC1(sa, rpm, drvDemandPct, actualPct) {
    let rpmRaw = rpmToRaw(rpm);
    let rpmLo = rpmRaw & 0xFF;
    let rpmHi = (rpmRaw >> 8) & 0xFF;

    let drvRaw = torquePctToRaw(drvDemandPct);
    let actRaw = torquePctToRaw(actualPct);

    // EEC1 de bancada:
    // byte 0 -> drv_dem_tq
    // byte 1 -> act_tq
    // byte 3-4 -> rpm
    // bytes 2, 5, 6 e 7 -> nao utilizados nesta versao
    can.sendFrame(0, 0x0CF00400 | (sa & 0xFF), 8, [
        drvRaw,
        actRaw,
        0xFF,
        rpmLo,
        rpmHi,
        0xFF,
        0xFF,
        0xFF
    ]);
}

function sendDM1SingleDTC(sa, spn, fmi) {
    let spnLow  = spn & 0xFF;
    let spnMid  = (spn >> 8) & 0xFF;
    let spnHigh = (spn >> 16) & 0x07;

    let b4 = ((spnHigh & 0x07) << 5) | (fmi & 0x1F);
    let b5 = 0x00; // CM = 0; occurrence count nao utilizado nesta versao

    can.sendFrame(0, 0x18FECA00 | (sa & 0xFF), 8, [
        0x04, // lamp simples para bancada
        0x00,
        spnLow,
        spnMid,
        b4,
        b5,
        0xFF,
        0xFF
    ]);
}

function scheduleNextFault() {
    // Aguarda pelo menos 5 min de operacao normal antes da proxima falha,
    // garantindo nova janela PRE completa.
    nextFaultAtMs = elapsedMs + MIN_NORMAL_BETWEEN_FAULTS_MS + randInt(EXTRA_NORMAL_MIN_MS, EXTRA_NORMAL_MAX_MS);
}

function updateBaseline() {
    // pequenas variacoes normais
    baseRpm = clamp(baseRpm + randInt(-40, 40), 2800, 3200);
    baseDrv = clamp(baseDrv + randInt(-2, 2), 25, 45);
    baseAct = clamp(baseAct + randInt(-2, 2), 20, 40);
}

function pickFault() {
    let faults = [
        {
            name: "RPM_ERRATIC",
            spn: 190,
            fmi: 2,
            build: function() {
                return {
                    rpm: randInt(0, 10),
                    drv: clamp(baseDrv, 25, 45),
                    act: -90
                };
            }
        },
        {
            name: "ACTUAL_TORQUE_ERRATIC",
            spn: 513,
            fmi: 2,
            build: function() {
                return {
                    rpm: clamp(baseRpm, 2800, 3200),
                    drv: clamp(baseDrv, 30, 45),
                    act: -80
                };
            }
        },
        {
            name: "DRIVER_DEMAND_STUCK_HIGH",
            spn: 512,
            fmi: 9,
            build: function() {
                return {
                    rpm: randInt(2200, 2600),
                    drv: 95,
                    act: 20
                };
            }
        }
    ];

    return faults[randInt(0, faults.length - 1)];
}

function startFault() {
    currentFault = pickFault();
    faultActive = true;
    faultEndMs = elapsedMs + FAULT_DURATION_MS;

    host.log("INICIO FALHA: " + currentFault.name +
             " | spn=" + currentFault.spn +
             " | fmi=" + currentFault.fmi +
             " | duracao_ms=" + FAULT_DURATION_MS);
}

function stopFault() {
    faultActive = false;
    host.log("FIM FALHA: " + currentFault.name);
    currentFault = null;
    scheduleNextFault();
}

function tick() {
    elapsedMs = tickCount * tickMs;

    // baseline muda devagar, 1 vez por segundo
    if (tickCount % 5 === 0 && !faultActive) {
        updateBaseline();
    }

    if (!faultActive && elapsedMs >= nextFaultAtMs) {
        startFault();
    }

    if (faultActive && elapsedMs >= faultEndMs) {
        stopFault();
    }

    let rpm = baseRpm;
    let drv = baseDrv;
    let act = baseAct;

    if (faultActive && currentFault !== null) {
        let frame = currentFault.build();
        rpm = frame.rpm;
        drv = frame.drv;
        act = frame.act;
    }

    sendEEC1(sa, rpm, drv, act);

    // durante a falha, envia DM1 a cada 1 s
    if (faultActive && currentFault !== null && (tickCount % 5 === 0)) {
        sendDM1SingleDTC(sa, currentFault.spn, currentFault.fmi);
    }

    if (tickCount % 5 === 0) {
        host.log(
            "t_s=" + Math.round(elapsedMs / 1000) +
            " | rpm=" + rpm +
            " | drv_dem_tq=" + drv +
            " | act_tq=" + act +
            (faultActive && currentFault !== null
                ? (" | fault=" + currentFault.name)
                : " | fault=NONE")
        );
    }

    tickCount++;
}
