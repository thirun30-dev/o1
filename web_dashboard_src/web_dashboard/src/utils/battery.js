// Battery helpers for /amr/battery_state. Missing values (null/NaN) are
// shown as "--". The robot has no battery monitor yet, so the system
// manager currently publishes a simulated battery.

const isNum = (v) => typeof v === 'number' && Number.isFinite(v);

export function batteryView(battery) {
  const pctKnown = isNum(battery?.percentage);
  const voltKnown = isNum(battery?.voltage) && battery.voltage > 1;
  const currKnown = isNum(battery?.current);
  const pct = pctKnown ? Math.round(battery.percentage * 100) : null;
  return {
    known: pctKnown || voltKnown,
    pct,
    pctText: pct === null ? '--' : `${pct}`,
    volt: voltKnown ? battery.voltage : null,
    voltText: voltKnown ? battery.voltage.toFixed(1) : '--',
    current: currKnown ? battery.current : null,
    currentText: currKnown ? battery.current.toFixed(1) : '--',
    level: pct === null ? 'UNKNOWN' : pct <= 10 ? 'CRITICAL' : pct <= 25 ? 'LOW' : 'NOMINAL',
  };
}
