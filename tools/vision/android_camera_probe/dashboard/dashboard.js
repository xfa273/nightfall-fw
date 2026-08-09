"use strict";

const csrfToken = document.querySelector(
  'meta[name="nightfall-dashboard-token"]',
).content;

const elements = Object.fromEntries([
  "connectionBadge", "connectionText", "hero", "heroKicker", "heroTitle",
  "heroDetail", "sessionRuns", "completedRuns", "savedRuns", "pendingRuns",
  "freshness", "validationBadge", "validationMessage", "latestRunName",
  "latestRunSize", "latestRunTransfer", "operationMessage", "deviceInfo",
  "startButton", "finishRunButton", "stopButton", "collectButton",
  "stopCollectButton",
  "eventList", "clearEventsButton",
].map((id) => [id, document.getElementById(id)]));

let previousCompleted = null;
let previousSaved = null;
let previousState = null;
let recordingStartedAt = null;
let lastSaveDetectedAt = null;
let lastStatusAt = null;
let lastOperationSequence = 0;
let pollInFlight = false;

function formatBytes(bytes) {
  if (!Number.isFinite(bytes) || bytes <= 0) return "—";
  const units = ["B", "KiB", "MiB", "GiB"];
  let value = bytes;
  let unit = units[0];
  for (let index = 1; index < units.length && value >= 1024; index += 1) {
    value /= 1024;
    unit = units[index];
  }
  return `${value.toFixed(value >= 10 ? 1 : 2)} ${unit}`;
}

function clockText(timestamp = Date.now()) {
  return new Intl.DateTimeFormat("ja-JP", {
    hour: "2-digit", minute: "2-digit", second: "2-digit",
  }).format(timestamp);
}

function addEvent(message) {
  const empty = elements.eventList.querySelector(".empty-event");
  if (empty) empty.remove();
  const item = document.createElement("li");
  const time = document.createElement("time");
  const text = document.createElement("span");
  time.textContent = clockText();
  text.textContent = message;
  item.append(time, text);
  elements.eventList.prepend(item);
}

function setHero(kind, kicker, title, detail) {
  elements.hero.className = `hero ${kind}`;
  elements.heroKicker.textContent = kicker;
  elements.heroTitle.textContent = title;
  elements.heroDetail.textContent = detail;
}

function setOnline(online, text) {
  elements.connectionBadge.className = `connection ${online ? "online" : "offline"}`;
  elements.connectionText.textContent = text;
}

function renderLatest(latest, inventoryError) {
  if (inventoryError) {
    elements.validationBadge.className = "pill bad";
    elements.validationBadge.textContent = "確認失敗";
    elements.validationMessage.textContent = inventoryError;
    return;
  }
  if (!latest) {
    elements.validationBadge.className = "pill neutral";
    elements.validationBadge.textContent = "動画なし";
    elements.validationMessage.textContent = "Pixel内に保存済み動画はありません。";
    elements.latestRunName.textContent = "—";
    elements.latestRunSize.textContent = "—";
    elements.latestRunTransfer.textContent = "—";
    return;
  }
  elements.latestRunName.textContent = latest.name;
  elements.latestRunSize.textContent = formatBytes(latest.video_bytes);
  elements.latestRunTransfer.textContent = latest.acknowledged
    ? "Mac受領・検証済み"
    : "Pixelのみ（未転送）";
  if (latest.complete) {
    elements.validationBadge.className = "pill good";
    elements.validationBadge.textContent = "保存完了";
    elements.validationMessage.textContent =
      "MP4・撮影結果・タイムスタンプ・reportの4ファイルが揃っています。";
  } else {
    const missing = latest.missing_artifacts.length
      ? `不足: ${latest.missing_artifacts.join(", ")}`
      : `status=${latest.status}`;
    elements.validationBadge.className = "pill bad";
    elements.validationBadge.textContent = "保存不完全";
    elements.validationMessage.textContent =
      `直近runを次の調整に使用できません。${missing}`;
  }
}

function renderOperation(operation) {
  if (!operation || operation.sequence === 0) {
    elements.operationMessage.className = "operation-message";
    elements.operationMessage.textContent = "操作待ち";
    return false;
  }
  elements.operationMessage.className = `operation-message ${operation.state}`;
  elements.operationMessage.textContent = operation.message;
  if (operation.sequence !== lastOperationSequence && operation.state !== "running") {
    addEvent(operation.message);
    lastOperationSequence = operation.sequence;
  }
  return operation.state === "running";
}

function renderOffline(data) {
  lastStatusAt = data.checked_at_unix_ms || Date.now();
  setOnline(false, "Pixelオフライン");
  setHero(
    "offline",
    "CONNECTION ERROR",
    "Pixelに接続できません",
    data.error || "PixelでNightfall HFR Recorderを開いてください。",
  );
  ["sessionRuns", "completedRuns", "savedRuns", "pendingRuns"].forEach((name) => {
    elements[name].textContent = "—";
  });
  const running = renderOperation(data.operation);
  elements.startButton.disabled = true;
  elements.finishRunButton.disabled = true;
  elements.stopButton.disabled = true;
  elements.collectButton.disabled = true;
  elements.stopCollectButton.disabled = true;
  if (!running && previousState !== "offline") addEvent("Pixelとの接続が切れました");
  previousState = "offline";
}

function renderOnline(data) {
  lastStatusAt = data.checked_at_unix_ms || Date.now();
  const pixel = data.pixel;
  const capture = data.capture;
  const state = capture.state;
  const completed = Number(capture.completed_runs || 0);
  const saved = Number(pixel.saved_runs || 0);
  const acknowledged = Number(pixel.acknowledged_runs || 0);
  const pending = Math.max(0, saved - acknowledged);
  const latest = data.latest_run;
  const justSaved = previousCompleted !== null && completed > previousCompleted;

  if (justSaved) {
    lastSaveDetectedAt = Date.now();
    const count = completed - previousCompleted;
    addEvent(`${count}本の保存完了を検出（セッション計${completed}本）`);
  } else if (previousSaved !== null && saved > previousSaved) {
    lastSaveDetectedAt = Date.now();
    addEvent(`Pixelの保存数が${saved}本へ増加`);
  }

  if (state === "recording") {
    if (previousState !== "recording" || recordingStartedAt === null) {
      recordingStartedAt = Date.now();
      addEvent("START信号を検出し、録画を開始");
    }
  } else {
    recordingStartedAt = null;
  }

  const localRecordingSeconds = recordingStartedAt === null
    ? 0
    : (Date.now() - recordingStartedAt) / 1000;
  const recordingSeconds = Math.max(
    localRecordingSeconds,
    Number(data.recording_elapsed_seconds || 0),
  );
  const warningSeconds = Number(data.recording_warning_seconds || 15);
  const latestComplete = latest && latest.complete;
  const recentSave = lastSaveDetectedAt !== null
    && Date.now() - lastSaveDetectedAt < 12000;

  if (state === "recording" && recordingSeconds >= warningSeconds) {
    setHero(
      "warning",
      "STOP SIGNAL WARNING",
      "録画が長時間継続中",
      `${Math.floor(recordingSeconds)}秒録画しています。STOP信号の見逃しや走行連結を確認してください。`,
    );
  } else if (state === "recording") {
    setHero(
      "recording",
      "RECORDING 240 FPS",
      "走行を録画中",
      "走行終了後の4パルスSTOP信号を待っています。",
    );
  } else if (state === "rearming") {
    setHero(
      latestComplete ? "saved" : "transition",
      latestComplete ? "SAVE VERIFIED" : "SAVING",
      latestComplete ? "動画を保存しました" : "保存を確認中",
      latestComplete
        ? `${completed}本目のファイル一式を確認しました。2秒後に再待機します。`
        : "Pixelが次の撮影に向けてカメラを再準備しています。",
    );
  } else if (state === "armed") {
    setHero(
      recentSave && latestComplete ? "saved" : "armed",
      recentSave && latestComplete ? "SAVE VERIFIED / ARMED" : "ARMED 240 FPS",
      recentSave && latestComplete ? "保存確認済み・次の走行OK" : "次の走行OK",
      `LED START信号を待機中です。現在のセッションで${completed}本保存済みです。`,
    );
  } else if (state === "finishing-run") {
    setHero(
      "transition",
      "SAVING CURRENT RUN",
      "現在の動画を保存中",
      capture.message || "保存後も連続撮影スタンバイを継続します。",
    );
  } else if (state === "starting" || state === "stopping") {
    setHero(
      "transition",
      state === "starting" ? "STARTING CAMERA" : "STOPPING CAMERA",
      state === "starting" ? "カメラを準備中" : "待機を終了中",
      capture.message || "Pixelの処理完了を待っています。",
    );
  } else if (state === "error") {
    setHero("error", "RECORDER ERROR", "撮影エラー", capture.message || "不明なエラー");
  } else {
    setHero(
      "transition",
      "STANDBY STOPPED",
      "撮影待機は停止中",
      capture.message || "「連続待機を開始」で撮影準備を始めます。",
    );
  }

  setOnline(true, `${pixel.model || "Pixel"} 接続中`);
  elements.sessionRuns.textContent = completed;
  elements.completedRuns.textContent = completed;
  elements.savedRuns.textContent = saved;
  elements.pendingRuns.textContent = pending;
  elements.deviceInfo.textContent =
    `${pixel.model || "Pixel"} / app ${pixel.app_version || "?"} / `
    + `${data.endpoint.host}:${data.endpoint.port}`;
  renderLatest(latest, data.inventory_error);

  const actionRunning = renderOperation(data.operation);
  const captureBusy = Boolean(pixel.capture_busy);
  const activeCapture = [
    "starting", "armed", "recording", "finishing-run", "rearming", "stopping",
  ].includes(state);
  elements.startButton.disabled = actionRunning || activeCapture;
  elements.finishRunButton.disabled = actionRunning
    || state !== "recording"
    || !capture.continuous_standby;
  elements.stopButton.disabled = actionRunning
    || !capture.continuous_standby
    || state === "finishing-run";
  elements.collectButton.disabled = actionRunning || captureBusy;
  elements.stopCollectButton.disabled = actionRunning
    || !captureBusy
    || state === "finishing-run";

  if (previousState !== state) {
    if (state === "armed" && previousState !== null) addEvent("次の走行の撮影スタンバイ完了");
    if (state === "error") addEvent(`Pixel撮影エラー: ${capture.message || "不明"}`);
  }
  previousCompleted = completed;
  previousSaved = saved;
  previousState = state;
}

async function pollStatus() {
  if (pollInFlight) return;
  pollInFlight = true;
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`Mac dashboard HTTP ${response.status}`);
    const data = await response.json();
    if (data.online) renderOnline(data);
    else renderOffline(data);
  } catch (error) {
    renderOffline({ checked_at_unix_ms: Date.now(), error: String(error) });
  } finally {
    pollInFlight = false;
  }
}

async function performAction(action) {
  const buttons = [
    elements.startButton, elements.finishRunButton, elements.stopButton,
    elements.collectButton, elements.stopCollectButton,
  ];
  buttons.forEach((button) => { button.disabled = true; });
  elements.operationMessage.className = "operation-message";
  elements.operationMessage.textContent = "操作を送信しています…";
  try {
    const response = await fetch(`/api/action/${action}`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Nightfall-Dashboard-Token": csrfToken,
      },
      body: "{}",
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || `HTTP ${response.status}`);
    elements.operationMessage.textContent = body.operation.message;
    await pollStatus();
  } catch (error) {
    elements.operationMessage.className = "operation-message error";
    elements.operationMessage.textContent = String(error);
    addEvent(`操作失敗: ${error}`);
  }
}

document.querySelectorAll("[data-action]").forEach((button) => {
  button.addEventListener("click", () => performAction(button.dataset.action));
});

elements.clearEventsButton.addEventListener("click", () => {
  elements.eventList.replaceChildren();
  const empty = document.createElement("li");
  empty.className = "empty-event";
  empty.textContent = "まだイベントはありません。";
  elements.eventList.append(empty);
});

setInterval(() => {
  if (lastStatusAt === null) {
    elements.freshness.textContent = "—";
    return;
  }
  elements.freshness.textContent = Math.max(
    0,
    Math.round((Date.now() - lastStatusAt) / 1000),
  );
}, 250);

setInterval(pollStatus, 750);
pollStatus();
