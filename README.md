# signalk-cpa-tcpa-plugin

SignalK plugin for maritime collision detection using CPA/TCPA analysis.

## Overview

This plugin monitors AIS targets and calculates collision risk using industry-standard **Closest Point of Approach (CPA)** and **Time to CPA (TCPA)** methods. When a vessel's predicted closest approach falls within configurable safety thresholds, the plugin triggers SignalK notifications.

## Installation

### Via SignalK Appstore

Search for `signalk-cpa-tcpa-plugin` in the SignalK Appstore and install.

### Manual

```bash
cd ~/.signalk/node_modules
npm install signalk-cpa-tcpa-plugin
```

Restart SignalK server after installation.

## Configuration

Configure via **Server > Plugin Config** in the SignalK web interface.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `safePassingDistanceMeters` | 500 | CPA threshold for alarm trigger |
| `alarmHysteresisMeters` | 200 | Buffer to prevent alarm flapping |
| `timeWindowMinutes` | 10 | Prediction horizon |
| `maxVesselSpeedMps` | 51.4 | Speed validation limit (100 knots) |
| `PosFreshBefore` | 600 | Maximum AIS data age (seconds) |

### Debug Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `debug.enabled` | false | Enable verbose debug logging |
| `debug.logInterval` | 60 | Status summary interval in seconds (0 = disabled) |
| `debug.logVesselDetails` | false | Include position/course/speed in debug output |

## How It Works

### Primary Detection: CPA/TCPA

For each AIS target with valid course and speed:

1. Calculate relative velocity vector between vessels
2. Compute time to closest point of approach (TCPA)
3. Determine minimum separation distance (CPA)
4. Compare against safety thresholds

**Alarm triggers when all conditions met:**
- CPA < safe passing distance
- TCPA < time window
- Vessels are converging

**No alarm for:**
- Diverging vessels (TCPA < 0)
- Safe passing distance maintained
- Parallel courses at constant separation

### Fallback Detection: Geometric Proximity

For targets missing course or speed data (anchored vessels, incomplete AIS):
- Uses 2x normal threshold as conservative buffer
- Position-only proximity check

### Hysteresis

Prevents rapid alarm cycling:
- Alarm ON: CPA < threshold
- Alarm OFF: CPA > threshold + hysteresis

## Notification Format

**Path:** `notifications.danger.collision`

**Active alarm:**
```json
{
  "method": ["visual", "sound"],
  "state": "alarm",
  "message": "CPA/TCPA collision warning - 1 threat(s)",
  "source": "signalk-cpa-tcpa-plugin",
  "threats": {
    "urn:mrn:imo:mmsi:123456789": {
      "method": "CPA",
      "cpaDistance": 450,
      "tcpaMinutes": 8.5,
      "relativeSpeed": 12.3,
      "bearing": 235,
      "distance": 850,
      "targetCourse": 180,
      "targetSpeed": 8.5,
      "position": { "latitude": 60.123, "longitude": 24.945 }
    }
  }
}
```

**Cleared:** `null`

## Subscribing

```javascript
{
  "context": "vessels.self",
  "subscribe": [{
    "path": "notifications.danger.collision",
    "format": "delta",
    "policy": "instant"
  }]
}
```

## Testing

```bash
node test.js
```

Validates distance calculations, bearing computations, and CPA/TCPA accuracy across 22 test scenarios.

## Technical Notes

**Performance:**
- Distance pre-filtering skips vessels beyond collision range (~33nm)
- Sub-millisecond per-vessel processing
- Bounded state with automatic cleanup (max 1000 vessels tracked)

**Data Requirements:**
- Required: Position (lat/lon)
- For CPA: Course over ground, speed over ground
- Optional: Vessel dimensions

**Algorithm:**
```
TCPA = -(relative_position . relative_velocity) / |relative_velocity|^2
CPA  = |relative_position + relative_velocity * TCPA|
```

## Safety Notice

This plugin is an advisory tool only. It does not replace proper lookout or compliance with COLREGS. Always maintain situational awareness and follow maritime regulations.

## License

MIT

## Author

Karl-Erik Gustafsson
