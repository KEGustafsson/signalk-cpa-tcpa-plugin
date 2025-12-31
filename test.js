#!/usr/bin/env node
/**
 * Collision Detector Test Suite
 * Tests the CPA/TCPA implementation (index.js)
 */

const assert = require('assert');

// Pure functions copied from index.js for isolated testing

// ============================================================================
// CONSTANTS (copied from implementation)
// ============================================================================

const CONSTANTS = {
	EARTH_RADIUS_METERS: 6371008.8,
	DEG_TO_RAD: Math.PI / 180,
	RAD_TO_DEG: 180 / Math.PI,
	MIN_RELATIVE_SPEED_MPS: 0.01,
};

// ============================================================================
// PURE FUNCTIONS (copied from index.js for testing)
// ============================================================================

function haversineDistance(from, to) {
	const φ1 = from.latitude * CONSTANTS.DEG_TO_RAD;
	const φ2 = to.latitude * CONSTANTS.DEG_TO_RAD;
	const Δφ = (to.latitude - from.latitude) * CONSTANTS.DEG_TO_RAD;
	const Δλ = (to.longitude - from.longitude) * CONSTANTS.DEG_TO_RAD;

	const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
			  Math.cos(φ1) * Math.cos(φ2) *
			  Math.sin(Δλ/2) * Math.sin(Δλ/2);
	const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

	return CONSTANTS.EARTH_RADIUS_METERS * c;
}

function computeForwardAzimuth(origin, destination) {
	if (!origin || !destination ||
		typeof origin.latitude !== 'number' || typeof origin.longitude !== 'number' ||
		typeof destination.latitude !== 'number' || typeof destination.longitude !== 'number') {
		return null;
	}

	const originLatRad = origin.latitude * CONSTANTS.DEG_TO_RAD;
	const destLatRad = destination.latitude * CONSTANTS.DEG_TO_RAD;
	const lonDiffRad = (destination.longitude - origin.longitude) * CONSTANTS.DEG_TO_RAD;

	const sinLonDiff = Math.sin(lonDiffRad);
	const cosLonDiff = Math.cos(lonDiffRad);
	const sinOriginLat = Math.sin(originLatRad);
	const cosOriginLat = Math.cos(originLatRad);
	const sinDestLat = Math.sin(destLatRad);
	const cosDestLat = Math.cos(destLatRad);

	const eastComponent = sinLonDiff * cosDestLat;
	const northComponent = cosOriginLat * sinDestLat - sinOriginLat * cosDestLat * cosLonDiff;

	const azimuthRad = Math.atan2(eastComponent, northComponent);
	return ((azimuthRad * CONSTANTS.RAD_TO_DEG) + 360) % 360;
}

function calculateCPA(vessel1, vessel2) {
	// Validate inputs - check for null, undefined, and NaN
	if (!vessel1.position || !vessel2.position) return null;
	if (vessel1.course === undefined || vessel1.course === null || isNaN(vessel1.course)) return null;
	if (vessel2.course === undefined || vessel2.course === null || isNaN(vessel2.course)) return null;
	if (vessel1.speed === undefined || vessel1.speed === null || isNaN(vessel1.speed)) return null;
	if (vessel2.speed === undefined || vessel2.speed === null || isNaN(vessel2.speed)) return null;

	// Convert to Cartesian velocities
	const v1x = vessel1.speed * Math.sin(vessel1.course);
	const v1y = vessel1.speed * Math.cos(vessel1.course);
	const v2x = vessel2.speed * Math.sin(vessel2.course);
	const v2y = vessel2.speed * Math.cos(vessel2.course);

	// Relative velocity (vessel2 in vessel1's frame)
	const relVelX = v2x - v1x;
	const relVelY = v2y - v1y;
	const relSpeed = Math.sqrt(relVelX * relVelX + relVelY * relVelY);

	// Relative position
	const dLon = (vessel2.position.longitude - vessel1.position.longitude) * CONSTANTS.DEG_TO_RAD;
	const dLat = (vessel2.position.latitude - vessel1.position.latitude) * CONSTANTS.DEG_TO_RAD;
	const avgLat = (vessel1.position.latitude + vessel2.position.latitude) / 2 * CONSTANTS.DEG_TO_RAD;

	const relPosX = dLon * Math.cos(avgLat) * CONSTANTS.EARTH_RADIUS_METERS;
	const relPosY = dLat * CONSTANTS.EARTH_RADIUS_METERS;

	// Same velocity case
	if (relSpeed < CONSTANTS.MIN_RELATIVE_SPEED_MPS) {
		const currentDist = Math.sqrt(relPosX * relPosX + relPosY * relPosY);
		return {
			cpaDistance: currentDist,
			tcpaSeconds: Infinity,
			diverging: false,
			relativeSpeed: 0,
			parallelCourse: true
		};
	}

	// TCPA calculation
	const tcpaSeconds = -(relPosX * relVelX + relPosY * relVelY) /
						 (relVelX * relVelX + relVelY * relVelY);

	if (tcpaSeconds < 0) {
		return {
			cpaDistance: Infinity,
			tcpaSeconds: 0,
			diverging: true,
			relativeSpeed: relSpeed,
			parallelCourse: false
		};
	}

	// CPA distance
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
// TEST UTILITIES
// ============================================================================

let testsRun = 0;
let testsPassed = 0;
let testsFailed = 0;

function test(name, fn) {
	testsRun++;
	try {
		fn();
		testsPassed++;
		console.log(`✓ ${name}`);
	} catch (error) {
		testsFailed++;
		console.log(`✗ ${name}`);
		console.log(`  Error: ${error.message}`);
		if (error.stack) {
			console.log(`  ${error.stack.split('\n')[1]}`);
		}
	}
}

function assertApprox(actual, expected, tolerance, message) {
	if (Math.abs(actual - expected) > tolerance) {
		throw new Error(`${message}: expected ${expected} ± ${tolerance}, got ${actual}`);
	}
}

// Conversion helpers
const knots2mps = (knots) => knots * 0.5144;
const nm2meters = (nm) => nm * 1852;
const deg2rad = (deg) => deg * Math.PI / 180;

// ============================================================================
// TEST SUITE
// ============================================================================

console.log('\n=== Collision Detector Test Suite ===\n');

// ----------------------------------------------------------------------------
// 1. Distance Calculation Tests
// ----------------------------------------------------------------------------

console.log('--- 1. Distance Calculation Tests ---');

test('1.1 Haversine: 1nm north', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 60.0 + (1/60), longitude: 24.0 };  // 1nm north
	const dist = haversineDistance(from, to);
	assertApprox(dist, 1852, 10, 'Distance');  // 1nm = 1852m
});

test('1.2 Haversine: 1nm east', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 60.0, longitude: 24.0 + (1/60) * (1/Math.cos(60 * Math.PI / 180)) };
	const dist = haversineDistance(from, to);
	assertApprox(dist, 1852, 20, 'Distance');
});

test('1.3 Haversine: Short distance accuracy', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 60.001, longitude: 24.001 };
	const dist = haversineDistance(from, to);
	// ~130m diagonal at 60°N
	assert(dist > 100 && dist < 200, `Distance should be ~130m, got ${dist}`);
});

test('1.4 Haversine: Long distance (100nm)', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 61.67, longitude: 24.0 };  // ~100nm north
	const dist = haversineDistance(from, to);
	assertApprox(dist, 185200, 500, 'Distance');  // 100nm = 185200m
});

test('1.5 Zero distance', () => {
	const pos = { latitude: 60.0, longitude: 24.0 };
	const dist = haversineDistance(pos, pos);
	assertApprox(dist, 0, 0.1, 'Distance');
});

// ----------------------------------------------------------------------------
// 2. Bearing Calculation Tests
// ----------------------------------------------------------------------------

console.log('\n--- 2. Bearing Calculation Tests ---');

test('2.1 Bearing: North (0°)', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 61.0, longitude: 24.0 };
	const bearing = computeForwardAzimuth(from, to);
	assertApprox(bearing, 0, 1, 'Bearing');
});

test('2.2 Bearing: East (90°)', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 60.0, longitude: 25.0 };
	const bearing = computeForwardAzimuth(from, to);
	assertApprox(bearing, 90, 1, 'Bearing');
});

test('2.3 Bearing: South (180°)', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 59.0, longitude: 24.0 };
	const bearing = computeForwardAzimuth(from, to);
	assertApprox(bearing, 180, 1, 'Bearing');
});

test('2.4 Bearing: West (270°)', () => {
	const from = { latitude: 60.0, longitude: 24.0 };
	const to = { latitude: 60.0, longitude: 23.0 };
	const bearing = computeForwardAzimuth(from, to);
	assertApprox(bearing, 270, 1, 'Bearing');
});

// ----------------------------------------------------------------------------
// 3. CPA/TCPA Tests - Critical Scenarios
// ----------------------------------------------------------------------------

console.log('\n--- 3. CPA/TCPA Tests ---');

test('3.1 Head-on collision course', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),   // North
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 60.0 + (1/60), longitude: 24.0 },  // 1nm north
		course: deg2rad(180), // South
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated');
	assert(!cpa.diverging, 'Vessels should be converging');
	assertApprox(cpa.cpaDistance, 0, 100, 'CPA distance');  // Near collision
	assertApprox(cpa.tcpaSeconds, 180, 10, 'TCPA');  // ~3 minutes (1nm at 20kn combined)
});

test('3.2 Parallel courses (no collision)', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(90),  // East
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 60.01, longitude: 24.0 },  // 0.6nm north
		course: deg2rad(90),  // East (parallel)
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated');
	assert(!cpa.diverging, 'Not diverging (parallel)');
	assert(cpa.parallelCourse, 'Should detect parallel course');
	assert(cpa.tcpaSeconds === Infinity, 'TCPA should be infinity for parallel');
	// CPA distance should be approximately current distance
	const currentDist = haversineDistance(vessel1.position, vessel2.position);
	assertApprox(cpa.cpaDistance, currentDist, 50, 'CPA distance = current distance');
});

test('3.3 Diverging vessels', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),   // North
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 59.99, longitude: 24.0 },  // 0.6nm south
		course: deg2rad(180), // South (diverging)
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated');
	assert(cpa.diverging, 'Vessels should be diverging');
	assert(cpa.cpaDistance === Infinity, 'CPA distance should be Infinity');
});

test('3.4 Crossing collision (90° angle)', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(90),  // East
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 60.0 + (0.5/60), longitude: 24.0 + (0.5/60) * (1/Math.cos(60 * Math.PI / 180)) },  // 0.5nm NE
		course: deg2rad(180), // South
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated');
	assert(!cpa.diverging, 'Vessels should be converging');
	// CPA should be small (collision course)
	assert(cpa.cpaDistance < 200, `CPA distance should be < 200m, got ${cpa.cpaDistance}`);
});

test('3.5 Overtaking (same direction, different speeds)', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),   // North
		speed: knots2mps(15)  // Faster
	};
	const vessel2 = {
		position: { latitude: 60.0 + (0.2/60), longitude: 24.0 },  // 0.2nm ahead
		course: deg2rad(0),   // North (same direction)
		speed: knots2mps(10)  // Slower
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated');
	assert(!cpa.diverging, 'Vessels should be converging');
	// CPA depends on lateral offset (here it's zero, so should be close)
	assert(cpa.cpaDistance < 100, 'CPA distance should be small (overtaking on same track)');
	assert(cpa.tcpaSeconds > 0, 'TCPA should be positive (future overtake)');
});

test('3.6 Zero speed vessels (both stationary)', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),
		speed: 0
	};
	const vessel2 = {
		position: { latitude: 60.01, longitude: 24.0 },
		course: deg2rad(0),
		speed: 0
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated for stationary vessels');
	assert(cpa.parallelCourse, 'Should detect same velocity (zero)');
	assert(cpa.tcpaSeconds === Infinity, 'TCPA should be infinity');
	const currentDist = haversineDistance(vessel1.position, vessel2.position);
	assertApprox(cpa.cpaDistance, currentDist, 50, 'CPA = current distance');
});

test('3.7 Missing course data', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: undefined,  // Missing
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 60.01, longitude: 24.0 },
		course: deg2rad(180),
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa === null, 'CPA should return null for missing course');
});

test('3.8 Missing speed data', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),
		speed: undefined  // Missing
	};
	const vessel2 = {
		position: { latitude: 60.01, longitude: 24.0 },
		course: deg2rad(180),
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa === null, 'CPA should return null for missing speed');
});

test('3.9 Null course data', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: null,  // Null (different from undefined)
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 60.01, longitude: 24.0 },
		course: deg2rad(180),
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa === null, 'CPA should return null for null course');
});

test('3.10 Null speed data', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),
		speed: null  // Null (different from undefined)
	};
	const vessel2 = {
		position: { latitude: 60.01, longitude: 24.0 },
		course: deg2rad(180),
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa === null, 'CPA should return null for null speed');
});

// ----------------------------------------------------------------------------
// 4. Real-world Maritime Scenarios
// ----------------------------------------------------------------------------

console.log('\n--- 4. Maritime Scenarios ---');

test('4.1 Scenario: Vessels passing port-to-port (safe)', () => {
	const vessel1 = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),   // North
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 60.0 + (1/60), longitude: 24.0 + (0.5/60) * (1/Math.cos(60 * Math.PI / 180)) },  // 1nm N, 0.5nm E
		course: deg2rad(180), // South
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should be calculated');
	assert(!cpa.diverging, 'Vessels converging');
	// CPA should be around 0.5nm (vessels pass with offset)
	const expectedCPA = nm2meters(0.5);
	assertApprox(cpa.cpaDistance, expectedCPA, 200, 'CPA distance');
});

test('4.2 Scenario: Approaching anchored vessel', () => {
	const moving = {
		position: { latitude: 60.0, longitude: 24.0 },
		course: deg2rad(0),
		speed: knots2mps(5)
	};
	const anchored = {
		position: { latitude: 60.0 + (0.1/60), longitude: 24.0 },
		course: deg2rad(0),  // Arbitrary (stationary)
		speed: 0
	};

	const cpa = calculateCPA(moving, anchored);
	assert(cpa !== null, 'CPA should be calculated');
	// Moving vessel heading directly at stationary vessel
	assert(cpa.cpaDistance < 100, 'CPA should be small (collision course with anchored vessel)');
});

test('4.3 Scenario: High latitude (75°N)', () => {
	const vessel1 = {
		position: { latitude: 75.0, longitude: 24.0 },
		course: deg2rad(90),  // East
		speed: knots2mps(10)
	};
	const vessel2 = {
		position: { latitude: 75.01, longitude: 24.0 },
		course: deg2rad(90),  // East (parallel)
		speed: knots2mps(10)
	};

	const cpa = calculateCPA(vessel1, vessel2);
	assert(cpa !== null, 'CPA should work at high latitude');
	assert(cpa.parallelCourse, 'Should detect parallel course');
	// Distance calculation should still be accurate
	const currentDist = haversineDistance(vessel1.position, vessel2.position);
	assertApprox(cpa.cpaDistance, currentDist, 100, 'CPA distance accurate at high latitude');
});

// ----------------------------------------------------------------------------
// Test Summary
// ----------------------------------------------------------------------------

console.log('\n=== Test Summary ===');
console.log(`Total: ${testsRun}`);
console.log(`Passed: ${testsPassed} ✓`);
console.log(`Failed: ${testsFailed} ✗`);

if (testsFailed === 0) {
	console.log('\n✓ All tests passed!');
	process.exit(0);
} else {
	console.log(`\n✗ ${testsFailed} test(s) failed`);
	process.exit(1);
}
