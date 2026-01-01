/*
 * Collision Detector - SignalK Plugin
 * Clean implementation with CPA/TCPA primary detection
 *
 * Architecture:
 * - Event-driven: subscribes to OTHER vessels' position changes only
 * - Direct data access: uses app.getPath() to fetch vessel data on demand
 * - Bounded state: tracks up to 1000 vessels with automatic cleanup
 * - CPA/TCPA as primary collision detection
 * - Geometric zones as fallback for vessels without course/speed
 * - Proper hysteresis to prevent alarm flapping
 * - Testable pure functions
 */

module.exports = function (app) {
const plugin = {};

plugin.id = 'signalk-cpa-tcpa-plugin';
plugin.name = 'CPA/TCPA Collision Detector';
plugin.description = 'SignalK plugin for CPA/TCPA collision detection of AIS vessels';

plugin.schema = {
	title: plugin.name,
	type: 'object',
	properties: {
		safePassingDistanceMeters: {
			type: 'number',
			title: 'Safe passing distance (meters)',
			description: 'Minimum CPA distance to trigger collision alarm',
			default: 500
		},
		alarmHysteresisMeters: {
			type: 'number',
			title: 'Alarm hysteresis (meters)',
			description: 'Additional distance for alarm-off threshold (prevents flapping)',
			default: 200
		},
		timeWindowMinutes: {
			type: 'number',
			title: 'Time window for collision prediction (minutes)',
			description: 'How far ahead to predict collisions',
			default: 10
		},
		rangeNauticalMiles: {
			type: 'number',
			title: 'Detection range (nautical miles)',
			description: 'Vessels beyond this range are ignored. Set based on your typical cruising area.',
			default: 10
		},
		timeouts: {
			type: 'object',
			title: 'Data freshness timeouts',
			properties: {
				PosFreshBefore: {
					type: 'number',
					title: 'Maximum data age (seconds)',
					description: 'AIS data older than this is considered stale',
					default: 600
				}
			}
		},
		debug: {
			type: 'object',
			title: 'Debug options',
			properties: {
				enabled: {
					type: 'boolean',
					title: 'Enable verbose debug logging',
					description: 'Log detailed information about collision detection',
					default: false
				},
				logInterval: {
					type: 'number',
					title: 'Status log interval (seconds)',
					description: 'How often to log status summary (0 = disabled)',
					default: 60
				},
				logVesselDetails: {
					type: 'boolean',
					title: 'Log vessel details',
					description: 'Include position/course/speed in debug output',
					default: false
				}
			}
		}
	}
};

// ============================================================================
// DEBUG UTILITIES
// ============================================================================

/**
 * Format position for debug output
 */
function formatPosition(pos) {
	if (!pos) return 'null';
	const lat = pos.latitude;
	const lon = pos.longitude;
	if (lat == null || lon == null) return '?';
	const latDir = lat >= 0 ? 'N' : 'S';
	const lonDir = lon >= 0 ? 'E' : 'W';
	return `${Math.abs(lat).toFixed(5)}°${latDir}, ${Math.abs(lon).toFixed(5)}°${lonDir}`;
}

/**
 * Format speed for debug output (m/s to knots)
 * Note: Uses hardcoded conversion as GEO constant not yet defined at this point
 */
function formatSpeed(speedMps) {
	if (speedMps === undefined || speedMps === null) return '?kn';
	const KNOTS_TO_MPS = 0.5144;
	return `${(speedMps / KNOTS_TO_MPS).toFixed(1)}kn`;
}

/**
 * Format course for debug output (radians to degrees)
 */
function formatCourse(courseRad) {
	if (courseRad === undefined || courseRad === null) return '?°';
	return `${(courseRad * 180 / Math.PI).toFixed(0)}°`;
}

/**
 * Format distance for debug output
 * Note: Uses hardcoded conversion as GEO constant not yet defined at this point
 */
function formatDistance(meters) {
	if (meters === undefined || meters === null) return '?';
	const NM_TO_METERS = 1852;
	if (meters < 1000) return `${meters.toFixed(0)}m`;
	return `${(meters / NM_TO_METERS).toFixed(2)}nm`;
}

/**
 * Format time for debug output
 */
function formatTime(seconds) {
	if (seconds === undefined || seconds === null) return '?';
	if (seconds === Infinity) return '∞';
	if (seconds < 60) return `${seconds.toFixed(0)}s`;
	return `${(seconds / 60).toFixed(1)}min`;
}

// ============================================================================
// GEODETIC AND DETECTION PARAMETERS
// ============================================================================

const GEO = {
	// WGS84 mean radius in meters (differs from simple 6371km approximation)
	MEAN_RADIUS_M: 6371008.8,
	// Angular conversion
	ANGLE_TO_RAD: Math.PI / 180,
	TO_DEGREES: 180 / Math.PI,
	// Unit conversions
	KNOTS_TO_MPS: 0.5144,
	NM_TO_METERS: 1852
};

const DETECTION = {
	// CPA calculation threshold
	RELATIVE_VELOCITY_FLOOR: 0.01,       // m/s minimum for valid CPA

	// Position validation bounds
	MAX_VESSEL_SPEED_MPS: 30,            // 30 m/s ≈ 58 knots, reasonable max for filtering
	JUMP_SPEED_FACTOR: 1.5,              // Multiplier for max allowed implied speed
	LAT_BOUND: 90,
	LON_BOUND: 180,

	// Resource management
	VESSEL_TRACKING_LIMIT: 1000,
	CLEANUP_FREQUENCY: 100               // Cleanup every N position checks
};

// ============================================================================
// PURE UTILITY FUNCTIONS (No side effects, testable)
// ============================================================================

/**
 * Calculate distance using Haversine formula (accurate, all distances)
 */
function haversineDistance(from, to) {
	if (!from || !to ||
		typeof from.latitude !== 'number' || typeof from.longitude !== 'number' ||
		typeof to.latitude !== 'number' || typeof to.longitude !== 'number') {
		return NaN;
	}

	const lat1Rad = from.latitude * GEO.ANGLE_TO_RAD;
	const lat2Rad = to.latitude * GEO.ANGLE_TO_RAD;
	const deltaLat = (to.latitude - from.latitude) * GEO.ANGLE_TO_RAD;
	const deltaLon = (to.longitude - from.longitude) * GEO.ANGLE_TO_RAD;

	const halfDeltaLatSin = Math.sin(deltaLat / 2);
	const halfDeltaLonSin = Math.sin(deltaLon / 2);
	const haversineA = halfDeltaLatSin * halfDeltaLatSin +
			  Math.cos(lat1Rad) * Math.cos(lat2Rad) *
			  halfDeltaLonSin * halfDeltaLonSin;
	const angularDist = 2 * Math.atan2(Math.sqrt(haversineA), Math.sqrt(1 - haversineA));

	return GEO.MEAN_RADIUS_M * angularDist;
}

/**
 * Calculate distance between two positions using Haversine formula
 */
function calculateDistance(from, to) {
	if (!from || !to) return null;

	const dist = haversineDistance(from, to);
	return isNaN(dist) ? null : dist;
}

/**
 * Compute initial bearing (forward azimuth) from origin to destination
 * Uses spherical law of sines approach
 * Returns degrees [0, 360)
 */
function computeForwardAzimuth(origin, destination) {
	if (!origin || !destination ||
		typeof origin.latitude !== 'number' || typeof origin.longitude !== 'number' ||
		typeof destination.latitude !== 'number' || typeof destination.longitude !== 'number') {
		return null;
	}

	const originLatRad = origin.latitude * GEO.ANGLE_TO_RAD;
	const destLatRad = destination.latitude * GEO.ANGLE_TO_RAD;
	const lonDiffRad = (destination.longitude - origin.longitude) * GEO.ANGLE_TO_RAD;

	// Spherical trig: sin/cos components for azimuth
	const sinLonDiff = Math.sin(lonDiffRad);
	const cosLonDiff = Math.cos(lonDiffRad);
	const sinOriginLat = Math.sin(originLatRad);
	const cosOriginLat = Math.cos(originLatRad);
	const sinDestLat = Math.sin(destLatRad);
	const cosDestLat = Math.cos(destLatRad);

	const eastComponent = sinLonDiff * cosDestLat;
	const northComponent = cosOriginLat * sinDestLat - sinOriginLat * cosDestLat * cosLonDiff;

	const azimuthRad = Math.atan2(eastComponent, northComponent);
	// Normalize to [0, 360)
	return ((azimuthRad * GEO.TO_DEGREES) + 360) % 360;
}

/**
 * Calculate CPA (Closest Point of Approach) and TCPA (Time to CPA)
 * Returns: { cpaDistance, tcpaSeconds, diverging, relativeSpeed } or null
 */
function calculateCPA(vessel1, vessel2) {
	// Validate inputs - check for null, undefined, and NaN
	if (!vessel1.position || !vessel2.position) return null;
	if (vessel1.course === undefined || vessel1.course === null || isNaN(vessel1.course)) return null;
	if (vessel2.course === undefined || vessel2.course === null || isNaN(vessel2.course)) return null;
	if (vessel1.speed === undefined || vessel1.speed === null || isNaN(vessel1.speed)) return null;
	if (vessel2.speed === undefined || vessel2.speed === null || isNaN(vessel2.speed)) return null;

	// Validate position coordinates
	if (typeof vessel1.position.latitude !== 'number' || typeof vessel1.position.longitude !== 'number') return null;
	if (typeof vessel2.position.latitude !== 'number' || typeof vessel2.position.longitude !== 'number') return null;
	if (isNaN(vessel1.position.latitude) || isNaN(vessel1.position.longitude)) return null;
	if (isNaN(vessel2.position.latitude) || isNaN(vessel2.position.longitude)) return null;

	// Convert to Cartesian velocities (m/s)
	// Course is radians from north, clockwise
	const v1x = vessel1.speed * Math.sin(vessel1.course);
	const v1y = vessel1.speed * Math.cos(vessel1.course);
	const v2x = vessel2.speed * Math.sin(vessel2.course);
	const v2y = vessel2.speed * Math.cos(vessel2.course);

	// Relative velocity (vessel2 in vessel1's frame)
	const relVelX = v2x - v1x;
	const relVelY = v2y - v1y;
	const relSpeed = Math.sqrt(relVelX * relVelX + relVelY * relVelY);

	// Calculate relative position (vessel2 relative to vessel1)
	// Use proper signed deltas
	const dLon = (vessel2.position.longitude - vessel1.position.longitude) * GEO.ANGLE_TO_RAD;
	const dLat = (vessel2.position.latitude - vessel1.position.latitude) * GEO.ANGLE_TO_RAD;

	const avgLat = (vessel1.position.latitude + vessel2.position.latitude) / 2 * GEO.ANGLE_TO_RAD;

	// Convert to meters with proper signs
	const relPosX = dLon * Math.cos(avgLat) * GEO.MEAN_RADIUS_M;
	const relPosY = dLat * GEO.MEAN_RADIUS_M;

	// Special case: vessels with same velocity
	if (relSpeed < DETECTION.RELATIVE_VELOCITY_FLOOR) {
		const currentDist = Math.sqrt(relPosX * relPosX + relPosY * relPosY);
		return {
			cpaDistance: currentDist,
			tcpaSeconds: Infinity,
			diverging: false,
			relativeSpeed: 0,
			parallelCourse: true
		};
	}

	// Time to CPA (seconds)
	// tcpa = -(relative position · relative velocity) / |relative velocity|²
	const tcpaSeconds = -(relPosX * relVelX + relPosY * relVelY) /
						 (relVelX * relVelX + relVelY * relVelY);

	// If TCPA is negative, vessels are diverging
	if (tcpaSeconds < 0) {
		return {
			cpaDistance: Infinity,
			tcpaSeconds: 0,
			diverging: true,
			relativeSpeed: relSpeed,
			parallelCourse: false
		};
	}

	// CPA position and distance
	const cpaX = relPosX + relVelX * tcpaSeconds;
	const cpaY = relPosY + relVelY * tcpaSeconds;
	const cpaDistance = Math.sqrt(cpaX * cpaX + cpaY * cpaY);

	return {
		cpaDistance: cpaDistance,
		tcpaSeconds: tcpaSeconds,
		diverging: false,
		relativeSpeed: relSpeed,
		parallelCourse: false
	};
}

// ============================================================================
// DATA VALIDATION
// ============================================================================

/**
 * Validate vessel update data BEFORE applying
 */
function validateVesselUpdate(vesselData, options) {
	const errors = [];

	// Position validation
	if (vesselData.position) {
		const lat = vesselData.position.latitude;
		const lon = vesselData.position.longitude;

		// Verify coordinates exist and are numbers
		if (typeof lat !== 'number' || isNaN(lat)) {
			errors.push(`Invalid latitude: ${lat}`);
		} else if (Math.abs(lat) > DETECTION.LAT_BOUND) {
			errors.push(`Latitude out of bounds: ${lat}`);
		}
		if (typeof lon !== 'number' || isNaN(lon)) {
			errors.push(`Invalid longitude: ${lon}`);
		} else if (Math.abs(lon) > DETECTION.LON_BOUND) {
			errors.push(`Longitude out of bounds: ${lon}`);
		}
	}

	// Speed validation
	if (vesselData.speed !== undefined && vesselData.speed !== null) {
		if (isNaN(vesselData.speed)) {
			errors.push(`Invalid speed: NaN`);
		} else if (vesselData.speed < 0) {
			errors.push(`Negative speed: ${vesselData.speed}`);
		}
	}

	// Course validation (should be radians 0-2π)
	if (vesselData.course !== undefined && vesselData.course !== null) {
		if (isNaN(vesselData.course)) {
			errors.push(`Invalid course: NaN`);
		} else if (vesselData.course < 0 || vesselData.course > 2 * Math.PI) {
			errors.push(`Course out of range: ${vesselData.course} radians`);
		}
	}

	return {
		valid: errors.length === 0,
		errors: errors
	};
}

/**
 * Detect position jumps (GPS glitches, data errors)
 */
function detectPositionJump(vesselData, previousData) {
	if (!previousData || !previousData.position || !previousData.timestamp) {
		return { jumped: false };
	}

	if (!vesselData.position || !vesselData.timestamp) {
		return { jumped: false };
	}

	const distance = haversineDistance(previousData.position, vesselData.position);
	if (isNaN(distance)) {
		return { jumped: false }; // Can't calculate - assume no jump
	}

	const timeDelta = (vesselData.timestamp - previousData.timestamp) / 1000; // seconds

	if (timeDelta <= 0) {
		return { jumped: false }; // No time elapsed or clock issue
	}

	const impliedSpeed = distance / timeDelta;
	const maxAllowedSpeed = DETECTION.MAX_VESSEL_SPEED_MPS * DETECTION.JUMP_SPEED_FACTOR;

	if (impliedSpeed > maxAllowedSpeed) {
		return {
			jumped: true,
			impliedSpeed: impliedSpeed,
			maxAllowedSpeed: maxAllowedSpeed,
			distance: distance,
			timeDelta: timeDelta
		};
	}

	return { jumped: false };
}

// ============================================================================
// COLLISION DETECTION STATE
// ============================================================================

/**
 * Create collision detector state manager
 */
function createCollisionDetector(app, options) {
	const state = {
		selfContext: null,    // Own vessel ID (e.g., 'self' or MMSI)
		selfFullContext: null, // Full context path (e.g., 'vessels.self')
		alarmActive: false,   // Current alarm state
		collisions: {},       // Current collision threats
		previousPositions: {}, // Track previous positions for jump detection
		callCount: 0,         // For deterministic cleanup scheduling
		statusLogTimer: null, // Timer for periodic status logging
		stats: {
			checksPerformed: 0,
			alarmsTriggered: 0,
			positionJumpsDetected: 0,
			cpaCalculations: 0,
			geometricFallbacks: 0,
			skippedStaleData: 0,
			skippedOutOfRange: 0,
			lastCleanup: Date.now()
		}
	};

	// Debug configuration
	const debugConfig = {
		enabled: options.debug?.enabled ?? false,
		logInterval: options.debug?.logInterval ?? 60,
		logVesselDetails: options.debug?.logVesselDetails ?? false
	};

	// ANSI color codes for vessel-specific debug output
	const VESSEL_COLORS = [
		'\x1b[36m',  // Cyan
		'\x1b[33m',  // Yellow
		'\x1b[35m',  // Magenta
		'\x1b[32m',  // Green
		'\x1b[34m',  // Blue
		'\x1b[91m',  // Bright Red
		'\x1b[92m',  // Bright Green
		'\x1b[93m',  // Bright Yellow
		'\x1b[94m',  // Bright Blue
		'\x1b[95m',  // Bright Magenta
		'\x1b[96m',  // Bright Cyan
	];
	const COLOR_RESET = '\x1b[0m';
	const vesselColorMap = {};

	/**
	 * Get consistent color for a vessel ID
	 */
	function getVesselColor(vesselId) {
		if (!vesselColorMap[vesselId]) {
			// Simple hash to assign color
			let hash = 0;
			for (let i = 0; i < vesselId.length; i++) {
				hash = ((hash << 5) - hash) + vesselId.charCodeAt(i);
				hash = hash & hash;
			}
			vesselColorMap[vesselId] = VESSEL_COLORS[Math.abs(hash) % VESSEL_COLORS.length];
		}
		return vesselColorMap[vesselId];
	}

	/**
	 * Debug logging helper - only logs when debug is enabled
	 */
	function debugLog(message, data = null) {
		if (!debugConfig.enabled) return;
		if (data) {
			app.debug(`[CPA] ${message}`, data);
		} else {
			app.debug(`[CPA] ${message}`);
		}
	}

	/**
	 * Debug logging with vessel color coding
	 */
	function debugLogVessel(vesselId, message) {
		if (!debugConfig.enabled) return;
		const color = getVesselColor(vesselId);
		app.debug(`[CPA] ${color}${vesselId}${COLOR_RESET}: ${message}`);
	}

	/**
	 * Log detailed vessel info with color coding
	 */
	function logVesselInfo(vesselId, vesselData) {
		if (!debugConfig.enabled || !debugConfig.logVesselDetails) return;
		if (!vesselData) {
			debugLogVessel(vesselId, `no data`);
			return;
		}
		const age = vesselData.timestamp ? ((Date.now() - vesselData.timestamp) / 1000).toFixed(0) : '?';
		debugLogVessel(vesselId, `pos=${formatPosition(vesselData.position)}, ` +
			`cog=${formatCourse(vesselData.course)}, ` +
			`sog=${formatSpeed(vesselData.speed)}, ` +
			`age=${age}s`);
	}

	/**
	 * Log periodic status summary
	 */
	function logStatusSummary() {
		const s = state.stats;
		app.debug(`[CPA] Status: ${s.checksPerformed} checks, ` +
			`${s.cpaCalculations} CPA calcs, ` +
			`${s.geometricFallbacks} fallbacks, ` +
			`${Object.keys(state.collisions).length} active threats, ` +
			`${Object.keys(state.previousPositions).length} vessels tracked`);

		if (state.alarmActive) {
			app.debug(`[CPA] ALARM ACTIVE - Threats: ${Object.keys(state.collisions).join(', ')}`);
		}

		if (debugConfig.enabled) {
			debugLog(`Extended stats: stale=${s.skippedStaleData}, ` +
				`outOfRange=${s.skippedOutOfRange}, ` +
				`jumps=${s.positionJumpsDetected}, ` +
				`alarms=${s.alarmsTriggered}`);
		}
	}

	/**
	 * Start periodic status logging
	 */
	function startStatusLogging() {
		if (debugConfig.logInterval > 0) {
			state.statusLogTimer = setInterval(logStatusSummary, debugConfig.logInterval * 1000);
			debugLog(`Status logging enabled every ${debugConfig.logInterval}s`);
		}
	}

	/**
	 * Stop periodic status logging
	 */
	function stopStatusLogging() {
		if (state.statusLogTimer) {
			clearInterval(state.statusLogTimer);
			state.statusLogTimer = null;
		}
	}

	/**
	 * Get vessel data directly from SignalK using app.getPath
	 */
	function getVesselData(vesselContext) {
		try {
			const position = app.getPath(`${vesselContext}.navigation.position`);
			const courseOverGround = app.getPath(`${vesselContext}.navigation.courseOverGroundTrue`);
			const headingTrue = app.getPath(`${vesselContext}.navigation.headingTrue`);
			const speedOverGround = app.getPath(`${vesselContext}.navigation.speedOverGround`);
			const length = app.getPath(`${vesselContext}.design.length`);
			const beam = app.getPath(`${vesselContext}.design.beam`);

			// Extract values (getPath returns objects with 'value' and 'timestamp')
			// Handle both wrapped {value, timestamp} and direct value formats
			// Use explicit null checks to avoid treating null as falsy and falling back incorrectly
			let posValue = null;
			if (position !== null && position !== undefined) {
				if (typeof position === 'object' && 'value' in position) {
					posValue = position.value;
				} else {
					posValue = position;
				}
			}

			// Extract course value - prefer COG, fallback to heading
			let courseValue = null;
			if (courseOverGround !== null && courseOverGround !== undefined) {
				courseValue = typeof courseOverGround === 'object' && 'value' in courseOverGround
					? courseOverGround.value
					: courseOverGround;
			}
			if (courseValue === null || courseValue === undefined) {
				if (headingTrue !== null && headingTrue !== undefined) {
					courseValue = typeof headingTrue === 'object' && 'value' in headingTrue
						? headingTrue.value
						: headingTrue;
				}
			}

			// Extract speed value
			let speedValue = null;
			if (speedOverGround !== null && speedOverGround !== undefined) {
				speedValue = typeof speedOverGround === 'object' && 'value' in speedOverGround
					? speedOverGround.value
					: speedOverGround;
			}

			// Parse timestamp safely, fallback to now if invalid
			let timestamp = Date.now();
			if (position?.timestamp) {
				const parsed = Date.parse(position.timestamp);
				if (!isNaN(parsed)) {
					timestamp = parsed;
				}
			}

			// Validate position exists and has valid coordinates
			if (!posValue || typeof posValue !== 'object') return null;
			if (typeof posValue.latitude !== 'number' || typeof posValue.longitude !== 'number') return null;
			if (isNaN(posValue.latitude) || isNaN(posValue.longitude)) return null;

			return {
				position: posValue,
				course: courseValue,
				speed: speedValue,
				length: length?.value?.overall || length?.overall || length?.value || length,
				beam: beam?.value || beam,
				timestamp: timestamp
			};
		} catch (error) {
			app.debug(`Error getting vessel data for ${vesselContext}: ${error.message}`);
			return null;
		}
	}

	/**
	 * Check if vessel data is fresh enough
	 */
	function isDataFresh(vesselData) {
		if (!vesselData || !vesselData.timestamp) return false;
		const age = (Date.now() - vesselData.timestamp) / 1000;
		// Allow small negative age (clock skew up to 60s) but reject very old data
		return age >= -60 && age <= options.timeouts.PosFreshBefore;
	}

	/**
	 * Validate vessel data before using in calculations
	 */
	function validateVesselData(vesselData) {
		if (!vesselData || !vesselData.position) return false;

		const validation = validateVesselUpdate(vesselData, options);
		return validation.valid;
	}

	/**
	 * Check for position jumps using stored previous position
	 */
	function checkPositionJump(vesselId, currentData) {
		const previousData = state.previousPositions[vesselId];

		if (previousData && currentData) {
			const jumpCheck = detectPositionJump(currentData, previousData);
			if (jumpCheck.jumped) {
				state.stats.positionJumpsDetected++;
				app.debug(`Position jump detected for ${vesselId}: ${jumpCheck.impliedSpeed.toFixed(1)} m/s`);
				return true;
			}
		}

		// Store current position for next check (with memory limit)
		if (currentData && currentData.position) {
			// Enforce memory limit before adding new entry
			const trackedCount = Object.keys(state.previousPositions).length;
			if (trackedCount >= DETECTION.VESSEL_TRACKING_LIMIT && !state.previousPositions[vesselId]) {
				// At limit and this is a new vessel - run cleanup first
				cleanupStalePositions();
				// If still at limit, skip tracking this vessel
				if (Object.keys(state.previousPositions).length >= DETECTION.VESSEL_TRACKING_LIMIT) {
					app.debug(`Memory limit reached (${DETECTION.VESSEL_TRACKING_LIMIT} vessels), not tracking ${vesselId}`);
					return false;
				}
			}

			state.previousPositions[vesselId] = {
				position: currentData.position,
				timestamp: currentData.timestamp
			};
		}

		return false;
	}

	/**
	 * Check collision using CPA/TCPA (primary method)
	 */
	function checkCPACollision(selfVessel, targetVessel, vesselId) {
		const cpaResult = calculateCPA(selfVessel, targetVessel);
		state.stats.cpaCalculations++;

		if (!cpaResult) {
			debugLogVessel(vesselId, `CPA calc failed (missing course/speed data)`);
			return null;
		}

		// Skip diverging vessels
		if (cpaResult.diverging) {
			debugLogVessel(vesselId, `Diverging, relSpeed=${formatSpeed(cpaResult.relativeSpeed)}`);
			return null;
		}

		// Check if CPA is within danger zone
		const threshold = state.alarmActive ?
			(options.safePassingDistanceMeters + options.alarmHysteresisMeters) :
			options.safePassingDistanceMeters;

		const tcpaMinutes = cpaResult.tcpaSeconds / 60;

		debugLogVessel(vesselId, `CPA=${formatDistance(cpaResult.cpaDistance)}, ` +
			`TCPA=${formatTime(cpaResult.tcpaSeconds)}, ` +
			`relSpeed=${formatSpeed(cpaResult.relativeSpeed)}, ` +
			`threshold=${formatDistance(threshold)}, ` +
			`parallel=${cpaResult.parallelCourse}`);

		if (cpaResult.cpaDistance > threshold) {
			debugLogVessel(vesselId, `Safe - CPA > threshold`);
			return null; // Safe passing distance
		}

		// Check if TCPA is within time window
		if (tcpaMinutes > options.timeWindowMinutes && !cpaResult.parallelCourse) {
			debugLogVessel(vesselId, `Safe - TCPA beyond window (${tcpaMinutes.toFixed(1)}min > ${options.timeWindowMinutes}min)`);
			return null; // Too far in future
		}

		// Collision risk detected
		debugLogVessel(vesselId, `*** COLLISION RISK *** CPA=${formatDistance(cpaResult.cpaDistance)}, TCPA=${formatTime(cpaResult.tcpaSeconds)}`);

		return {
			method: 'CPA',
			cpaDistance: cpaResult.cpaDistance,
			tcpaMinutes: tcpaMinutes,
			relativeSpeed: cpaResult.relativeSpeed,
			parallelCourse: cpaResult.parallelCourse || false,
			bearing: computeForwardAzimuth(selfVessel.position, targetVessel.position),
			distance: calculateDistance(selfVessel.position, targetVessel.position),
			targetCourse: targetVessel.course != null ? targetVessel.course * GEO.TO_DEGREES : undefined,
			targetSpeed: targetVessel.speed
		};
	}

	/**
	 * Check collision using geometric zones (fallback method)
	 */
	function checkGeometricCollision(selfVessel, targetVessel, vesselId) {
		if (!selfVessel || !selfVessel.position || !targetVessel.position) return null;

		// Only use geometric method if CPA not available
		// (missing course or speed data - check for null, undefined, and NaN)
		const selfHasCourse = selfVessel.course != null && !isNaN(selfVessel.course) &&
							  selfVessel.speed != null && !isNaN(selfVessel.speed);
		const targetHasCourse = targetVessel.course != null && !isNaN(targetVessel.course) &&
								targetVessel.speed != null && !isNaN(targetVessel.speed);

		if (selfHasCourse && targetHasCourse) {
			return null; // Use CPA method instead
		}

		state.stats.geometricFallbacks++;

		// Calculate collision zones based on uncertainty
		const distance = calculateDistance(selfVessel.position, targetVessel.position);

		// Use conservative threshold for vessels with unknown motion
		const threshold = state.alarmActive ?
			(options.safePassingDistanceMeters * 2 + options.alarmHysteresisMeters) :
			(options.safePassingDistanceMeters * 2);

		const reason = !selfHasCourse ? 'own vessel missing COG/SOG' : 'target missing COG/SOG';
		debugLogVessel(vesselId, `Geometric fallback (${reason}), dist=${formatDistance(distance)}, threshold=${formatDistance(threshold)}`);

		if (distance != null && distance < threshold) {
			debugLogVessel(vesselId, `*** GEOMETRIC PROXIMITY ALERT *** dist=${formatDistance(distance)}`);
			return {
				method: 'GEOMETRIC',
				distance: distance,
				bearing: computeForwardAzimuth(selfVessel.position, targetVessel.position),
				reason: `Missing course/speed data (${reason}) - using conservative proximity check`
			};
		}

		return null;
	}

	/**
	 * Check collision for a specific target vessel (triggered by position update)
	 * Uses app.getPath for direct data access
	 */
	function checkCollisionForVessel(targetVesselContext) {
		// Validate context format
		if (!targetVesselContext || !targetVesselContext.includes('.')) {
			return;
		}

		const vesselId = targetVesselContext.split('.').pop();
		if (!vesselId || vesselId === 'vessels') {
			return; // Invalid vessel ID
		}

		// Track stats
		state.callCount++;
		state.stats.checksPerformed++;

		// Deterministic cleanup scheduling (every N calls)
		if (state.callCount % DETECTION.CLEANUP_FREQUENCY === 0) {
			cleanupStalePositions();
			cleanupStaleCollisions();
			debugLog(`Cleanup triggered at check #${state.callCount}`);
		}

		// Get own vessel data directly from SignalK
		const selfVessel = getVesselData(state.selfFullContext);
		if (!selfVessel) {
			// Log at debug level with own vessel context (not target vesselId)
			app.debug(`[CPA] ${state.selfContext}: Own vessel position data not available (no valid position from ${state.selfFullContext})`);
			return;
		}
		if (!isDataFresh(selfVessel)) {
			app.debug(`[CPA] ${state.selfContext}: Own vessel data stale (age=${((Date.now() - selfVessel.timestamp) / 1000).toFixed(0)}s, max=${options.timeouts.PosFreshBefore}s)`);
			return;
		}
		if (!validateVesselData(selfVessel)) {
			// Log detailed validation info to help diagnose the issue
			const validation = validateVesselUpdate(selfVessel, options);
			app.debug(`[CPA] ${state.selfContext}: Own vessel data validation failed: ${validation.errors.join(', ') || 'unknown error'}`);
			return;
		}

		logVesselInfo(state.selfContext, selfVessel);

		// Get target vessel data directly from SignalK
		const targetVessel = getVesselData(targetVesselContext);
		if (!targetVessel) {
			debugLogVessel(vesselId, `Target data not available`);
			removeCollision(vesselId);
			return;
		}
		if (!isDataFresh(targetVessel)) {
			state.stats.skippedStaleData++;
			debugLogVessel(vesselId, `Target data stale (age=${((Date.now() - targetVessel.timestamp) / 1000).toFixed(0)}s)`);
			removeCollision(vesselId);
			return;
		}
		if (!validateVesselData(targetVessel)) {
			debugLogVessel(vesselId, `Target data invalid`);
			removeCollision(vesselId);
			return;
		}

		logVesselInfo(vesselId, targetVessel);

		// Check for position jump
		if (checkPositionJump(vesselId, targetVessel)) {
			debugLogVessel(vesselId, `Skipped due to position jump`);
			return; // Skip this update due to position jump
		}

		// Distance pre-filter - skip vessels beyond configured range
		const distance = calculateDistance(selfVessel.position, targetVessel.position);

		if (distance === null || distance > options.rangeMeters) {
			state.stats.skippedOutOfRange++;
			debugLogVessel(vesselId, `Out of range (dist=${formatDistance(distance)}, max=${formatDistance(options.rangeMeters)})`);
			removeCollision(vesselId);
			return;
		}

		debugLogVessel(vesselId, `Processing - dist=${formatDistance(distance)}`);

		// Try CPA method first (primary)
		let collision = checkCPACollision(selfVessel, targetVessel, vesselId);

		// Fallback to geometric if CPA not available
		if (!collision) {
			collision = checkGeometricCollision(selfVessel, targetVessel, vesselId);
		}

		const wasTracking = !!state.collisions[vesselId];

		if (collision) {
			state.collisions[vesselId] = {
				...collision,
				vesselId: vesselId,
				position: targetVessel.position
			};
			if (!wasTracking) {
				debugLogVessel(vesselId, `Added to collision tracking`);
			}
		} else {
			if (wasTracking) {
				debugLogVessel(vesselId, `Removed from collision tracking`);
			}
			delete state.collisions[vesselId];
		}

		// Update alarm state with hysteresis
		updateAlarmState(Object.keys(state.collisions).length > 0);
	}

	/**
	 * Remove vessel from collisions and update alarm state
	 */
	function removeCollision(vesselId) {
		if (state.collisions[vesselId]) {
			delete state.collisions[vesselId];
			updateAlarmState(Object.keys(state.collisions).length > 0);
		}
	}

	/**
	 * Update alarm state with hysteresis
	 */
	function updateAlarmState(shouldBeActive) {
		if (shouldBeActive && !state.alarmActive) {
			// Turn ON
			state.alarmActive = true;
			publishCollisionNotification(true);
		} else if (!shouldBeActive && state.alarmActive) {
			// Turn OFF
			state.alarmActive = false;
			publishCollisionNotification(false);
		}
		// Otherwise maintain current state (hysteresis)
	}

	/**
	 * Publish collision notification to SignalK notification system
	 */
	function publishCollisionNotification(isActive) {
		const notificationPath = 'notifications.danger.collision';
		const currentTimestamp = new Date().toISOString();

		// Build notification value - use state:"normal" to clear instead of null
		// Setting to null can cause issues with SignalK data browser
		const notificationValue = isActive ? {
			method: ['visual', 'sound'],
			state: 'alarm',
			message: `CPA/TCPA collision warning - ${Object.keys(state.collisions).length} threat(s)`,
			source: plugin.id,
			threats: state.collisions
		} : {
			state: 'normal',
			message: 'No collision threats',
			source: plugin.id
		};

		const buildNotificationPayload = () => ({
			context: 'vessels.self',
			updates: [{
				values: [{
					path: notificationPath,
					value: notificationValue
				}],
				source: { label: plugin.id },
				timestamp: currentTimestamp
			}]
		});

		try {
			if (isActive) {
				state.stats.alarmsTriggered++;
			}
			app.handleMessage(plugin.id, buildNotificationPayload());
			app.debug(`Collision notification ${isActive ? 'ACTIVATED' : 'CLEARED'} - tracking ${Object.keys(state.collisions).length} vessel(s)`);
		} catch (err) {
			app.error(`Failed to publish collision notification: ${err.message}`);
		}
	}

	/**
	 * Reset detector state (for restart scenarios)
	 */
	function reset() {
		state.alarmActive = false;
		state.collisions = {};
		state.previousPositions = {};
		state.callCount = 0;
		state.stats = {
			checksPerformed: 0,
			alarmsTriggered: 0,
			positionJumpsDetected: 0,
			cpaCalculations: 0,
			geometricFallbacks: 0,
			skippedStaleData: 0,
			skippedOutOfRange: 0,
			lastCleanup: Date.now()
		};
		app.debug('Collision detector state reset');
	}

	/**
	 * Get current status for monitoring
	 */
	function getStatus() {
		return {
			alarmActive: state.alarmActive,
			activeCollisions: Object.keys(state.collisions).length,
			trackedVessels: Object.keys(state.previousPositions).length,
			stats: { ...state.stats },
			collisionDetails: { ...state.collisions }
		};
	}

	/**
	 * Clean up stale entries from previousPositions to prevent memory leak
	 */
	function cleanupStalePositions() {
		const now = Date.now();
		const maxAge = options.timeouts.PosFreshBefore * 1000 * 2; // 2x freshness timeout
		let cleaned = 0;

		for (const vesselId of Object.keys(state.previousPositions)) {
			const data = state.previousPositions[vesselId];
			if (data.timestamp && (now - data.timestamp) > maxAge) {
				delete state.previousPositions[vesselId];
				cleaned++;
			}
		}

		if (cleaned > 0) {
			state.stats.lastCleanup = now;
			app.debug(`Cleaned up ${cleaned} stale position entries`);
		}
	}

	/**
	 * Clean up stale collision entries (vessels no longer being tracked)
	 */
	function cleanupStaleCollisions() {
		let cleaned = 0;

		for (const vesselId of Object.keys(state.collisions)) {
			// Check if the vessel data is still fresh
			const vesselContext = `vessels.${vesselId}`;
			const vesselData = getVesselData(vesselContext);
			if (!vesselData || !isDataFresh(vesselData)) {
				delete state.collisions[vesselId];
				cleaned++;
			}
		}

		if (cleaned > 0) {
			app.debug(`Cleaned up ${cleaned} stale collision entries`);
			updateAlarmState(Object.keys(state.collisions).length > 0);
		}
	}

	return {
		checkCollisionForVessel,
		setSelfContext: (context, fullContext) => {
			state.selfContext = context;
			state.selfFullContext = fullContext;
		},
		getSelfContext: () => state.selfContext,
		getSelfFullContext: () => state.selfFullContext,
		getState: () => state,
		getStatus,
		reset,
		startStatusLogging,
		stopStatusLogging,
		isDebugEnabled: () => debugConfig.enabled
	};
}

// ============================================================================
// CONFIGURATION VALIDATION
// ============================================================================

/**
 * Validate and sanitize configuration options
 */
function validateOptions(options) {
	const errors = [];
	const warnings = [];

	// Validate numeric ranges (check for NaN since typeof NaN === 'number')
	if (options.safePassingDistanceMeters !== undefined) {
		if (typeof options.safePassingDistanceMeters !== 'number' ||
			isNaN(options.safePassingDistanceMeters) ||
			options.safePassingDistanceMeters <= 0) {
			errors.push('safePassingDistanceMeters must be a positive number');
		} else if (options.safePassingDistanceMeters < 50) {
			warnings.push('safePassingDistanceMeters < 50m is very aggressive');
		}
	}

	if (options.alarmHysteresisMeters !== undefined) {
		if (typeof options.alarmHysteresisMeters !== 'number' ||
			isNaN(options.alarmHysteresisMeters) ||
			options.alarmHysteresisMeters < 0) {
			errors.push('alarmHysteresisMeters must be a non-negative number');
		}
	}

	if (options.timeWindowMinutes !== undefined) {
		if (typeof options.timeWindowMinutes !== 'number' ||
			isNaN(options.timeWindowMinutes) ||
			options.timeWindowMinutes <= 0) {
			errors.push('timeWindowMinutes must be a positive number');
		} else if (options.timeWindowMinutes > 60) {
			warnings.push('timeWindowMinutes > 60 may cause excessive alerts');
		}
	}

	if (options.rangeNauticalMiles !== undefined) {
		if (typeof options.rangeNauticalMiles !== 'number' ||
			isNaN(options.rangeNauticalMiles) ||
			options.rangeNauticalMiles <= 0) {
			errors.push('rangeNauticalMiles must be a positive number');
		} else if (options.rangeNauticalMiles > 50) {
			warnings.push('rangeNauticalMiles > 50 may impact performance with many AIS targets');
		}
	}

	if (options.timeouts?.PosFreshBefore !== undefined) {
		if (typeof options.timeouts.PosFreshBefore !== 'number' ||
			isNaN(options.timeouts.PosFreshBefore) ||
			options.timeouts.PosFreshBefore <= 0) {
			errors.push('timeouts.PosFreshBefore must be a positive number');
		}
	}

	return { errors, warnings, valid: errors.length === 0 };
}

// ============================================================================
// PLUGIN LIFECYCLE MANAGEMENT
// ============================================================================

let detector = null;
let subscriptionCleanupFns = [];
let ownVesselId = null;

plugin.start = function (options) {
	// Validate configuration
	const validation = validateOptions(options);
	if (!validation.valid) {
		const errorMsg = `Configuration errors: ${validation.errors.join(', ')}`;
		app.error(errorMsg);
		app.setPluginError(errorMsg);
		return;
	}
	if (validation.warnings.length > 0) {
		validation.warnings.forEach(w => app.debug(`Config warning: ${w}`));
	}

	// Merge user options with defaults
	const mergedConfig = {
		safePassingDistanceMeters: options.safePassingDistanceMeters ?? 500,
		alarmHysteresisMeters: options.alarmHysteresisMeters ?? 200,
		timeWindowMinutes: options.timeWindowMinutes ?? 10,
		rangeMeters: (options.rangeNauticalMiles ?? 10) * 1852,  // Convert nm to meters
		timeouts: {
			PosFreshBefore: options.timeouts?.PosFreshBefore ?? 600
		},
		debug: {
			enabled: options.debug?.enabled ?? false,
			logInterval: options.debug?.logInterval ?? 60,
			logVesselDetails: options.debug?.logVesselDetails ?? false
		}
	};

	// Initialize detector
	detector = createCollisionDetector(app, mergedConfig);

	// Resolve own vessel identifier
	ownVesselId = app.selfId || 'self';
	const ownVesselContext = `vessels.${ownVesselId}`;
	detector.setSelfContext(ownVesselId, ownVesselContext);

	app.debug(`CPA/TCPA detector initialized. Own vessel: ${ownVesselId}`);
	app.debug(`Parameters: CPA threshold=${mergedConfig.safePassingDistanceMeters}m, ` +
		`hysteresis=${mergedConfig.alarmHysteresisMeters}m, ` +
		`TCPA window=${mergedConfig.timeWindowMinutes}min, ` +
		`range=${(mergedConfig.rangeMeters / 1852).toFixed(1)}nm`);

	if (mergedConfig.debug.enabled) {
		app.debug(`Debug mode ENABLED - verbose logging active`);
		if (mergedConfig.debug.logVesselDetails) {
			app.debug(`Vessel details logging ENABLED`);
		}
	}

	// Start periodic status logging
	detector.startStatusLogging();

	// Register position update subscription for all vessels
	// Detection triggers on position changes, fetches other data via app.getPath
	const positionSubscription = {
		context: 'vessels.*',
		subscribe: [{
			path: 'navigation.position',
			format: 'delta',
			policy: 'instant',
			minPeriod: 0
		}]
	};

	app.subscriptionmanager.subscribe(
		positionSubscription,
		subscriptionCleanupFns,
		subscriptionErr => {
			const errMsg = subscriptionErr?.message || String(subscriptionErr);
			app.error(`Position subscription failed: ${errMsg}`);
			app.setPluginError(`Subscription failure: ${errMsg}`);
		},
		processDeltaMessage
	);
};

plugin.stop = function () {
	// Log final stats before stopping
	if (detector) {
		// Stop periodic status logging
		detector.stopStatusLogging();

		const status = detector.getStatus();
		const s = status.stats;
		app.debug(`Shutdown stats: ${s.checksPerformed} checks, ` +
			`${s.cpaCalculations} CPA calcs, ` +
			`${s.geometricFallbacks} fallbacks, ` +
			`${s.alarmsTriggered} alarms, ` +
			`${s.positionJumpsDetected} jumps, ` +
			`${s.skippedStaleData} stale, ` +
			`${s.skippedOutOfRange} out-of-range`);

		// Clear any active collision notification
		if (status.alarmActive) {
			try {
				const clearPayload = {
					context: 'vessels.self',
					updates: [{
						values: [{
							path: 'notifications.danger.collision',
							value: {
								state: 'normal',
								message: 'Plugin stopped',
								source: plugin.id
							}
						}],
						source: { label: plugin.id },
						timestamp: new Date().toISOString()
					}]
				};
				app.handleMessage(plugin.id, clearPayload);
				app.debug('Cleared active collision notification on shutdown');
			} catch (err) {
				app.debug(`Could not clear notification on shutdown: ${err.message}`);
			}
		}

		// Reset detector state
		detector.reset();
	}

	// Cleanup all subscriptions
	for (const cleanupFn of subscriptionCleanupFns) {
		try { cleanupFn(); } catch (_) { /* ignore cleanup errors */ }
	}
	subscriptionCleanupFns = [];

	// Release references
	ownVesselId = null;
	detector = null;

	app.debug('CPA/TCPA detector shutdown complete');
};

/**
 * Plugin status for SignalK status API
 */
plugin.statusMessage = function () {
	if (!detector) {
		return 'Not running';
	}
	const status = detector.getStatus();
	if (status.alarmActive) {
		return `ALARM: ${status.activeCollisions} collision(s) detected`;
	}
	return `Monitoring ${status.trackedVessels} vessel(s), ${status.stats.checksPerformed} checks performed`;
};

/**
 * Process incoming SignalK delta messages for position updates
 * Triggers CPA/TCPA calculation when other vessels report new positions
 */
function processDeltaMessage(delta) {
	if (!detector || !delta.updates) return;

	const sourceContext = delta.context;
	if (!sourceContext) return;

	// Extract vessel identifier from context path
	const sourceVesselId = sourceContext.split('.').pop();

	// Filter out own vessel - only process other vessels' position changes
	const isOwnVessel = sourceVesselId === 'self' ||
		sourceVesselId === ownVesselId ||
		sourceContext === `vessels.${ownVesselId}`;

	if (isOwnVessel) return;

	// Verify this delta contains position data
	const containsPosition = delta.updates.some(update =>
		update.values?.some(v => v.path === 'navigation.position')
	);

	if (!containsPosition) return;

	// Initiate collision analysis for this vessel
	detector.checkCollisionForVessel(sourceContext);
}

return plugin;
};
