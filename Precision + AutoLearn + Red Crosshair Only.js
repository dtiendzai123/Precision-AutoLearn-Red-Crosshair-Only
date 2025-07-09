// =================== Vector3 Class ===================
class Vector3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  add(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
    return this;
  }

  subtract(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
    return this;
  }

  multiplyScalar(s) {
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
  }

  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

  normalize() {
    const len = this.length();
    if (len > 0) this.multiplyScalar(1 / len);
    return this;
  }

  clone() {
    return new Vector3(this.x, this.y, this.z);
  }

  set(x, y, z) {
    this.x = x; this.y = y; this.z = z;
    return this;
  }

  static zero() {
    return new Vector3(0, 0, 0);
  }
}

// =================== Optimized Kalman Filter ===================
class OptimizedKalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R;
    this.Q = Q;
    this.cov = 1.0;
    this.x = 0;
    this.initialized = false;
  }

  filter(z) {
    if (!this.initialized) {
      this.x = z;
      this.cov = this.R;
      this.initialized = true;
      return this.x;
    }
    const predCov = this.cov + this.Q;
    const K = predCov / (predCov + this.R);
    this.x += K * (z - this.x);
    this.cov = (1 - K) * predCov;
    return this.x;
  }

  reset() {
    this.cov = 1.0;
    this.x = 0;
    this.initialized = false;
  }
}

// =================== Quaternion to Matrix ===================
function quaternionToMatrix(q) {
  const x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
  const xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
  const yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
  const wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;

  return {
    e00: 1 - (yy + zz), e01: xy - wz, e02: xz + wy,
    e10: xy + wz, e11: 1 - (xx + zz), e12: yz - wx,
    e20: xz - wy, e21: yz + wx, e22: 1 - (xx + yy)
  };
}

// =================== Transform Bone Head ===================
function transformBoneHead(pos, rotation, scale, bindpose, velocity = null) {
  const rotM = quaternionToMatrix(rotation);
  let x = rotM.e00 * pos.x + rotM.e01 * pos.y + rotM.e02 * pos.z;
  let y = rotM.e10 * pos.x + rotM.e11 * pos.y + rotM.e12 * pos.z;
  let z = rotM.e20 * pos.x + rotM.e21 * pos.y + rotM.e22 * pos.z;
  x *= scale.x; y *= scale.y; z *= scale.z;

  const worldPos = {
    x: bindpose.e00 * x + bindpose.e01 * y + bindpose.e02 * z + bindpose.e03,
    y: bindpose.e10 * x + bindpose.e11 * y + bindpose.e12 * z + bindpose.e13,
    z: bindpose.e20 * x + bindpose.e21 * y + bindpose.e22 * z + bindpose.e23
  };

  if (velocity) {
    worldPos.x += velocity.x * 0.15;
    worldPos.y += velocity.y * 0.15;
    worldPos.z += velocity.z * 0.15;
  }

  return new Vector3(worldPos.x, worldPos.y, worldPos.z);
}

// =================== Precision Aim Helper ===================
class PrecisionAimHelper {
  constructor() {
    this.centerBias = 0.92;
    this.kalmanX = new OptimizedKalmanFilter(0.005, 0.00001);
    this.kalmanY = new OptimizedKalmanFilter(0.005, 0.00001);
    this.kalmanZ = new OptimizedKalmanFilter(0.005, 0.00001);
    this.buffer = [];
    this.maxSize = 5;
  }

  refine(current, target) {
    const dx = this.kalmanX.filter(target.x - current.x);
    const dy = this.kalmanY.filter(target.y - current.y);
    const dz = this.kalmanZ.filter(target.z - current.z);
    const correction = new Vector3(dx, dy, dz).multiplyScalar(this.centerBias);
    const refined = current.clone().add(correction);
    this.buffer.push(refined.clone());
    if (this.buffer.length > this.maxSize) this.buffer.shift();
    return this.smooth();
  }

  smooth() {
    if (this.buffer.length === 0) return Vector3.zero();
    let result = Vector3.zero();
    let totalWeight = 0;
    for (let i = 0; i < this.buffer.length; i++) {
      const w = (i + 1);
      result.add(this.buffer[i].clone().multiplyScalar(w));
      totalWeight += w;
    }
    return result.multiplyScalar(1 / totalWeight);
  }
}

// =================== AimLock System ===================
class AdvancedAimLockSystem {
  constructor() {
    this.velocity = Vector3.zero();
    this.prev = null;
    this.precision = new PrecisionAimHelper();
  }

  update(current) {
    const now = Date.now();
    if (this.prev) {
      const dt = (now - this.lastTime) / 1000;
      const vel = current.clone().subtract(this.prev).multiplyScalar(1 / dt);
      this.velocity = vel;
    }
    this.prev = current.clone();
    this.lastTime = now;
    const predicted = current.clone().add(this.velocity.clone().multiplyScalar(0.1));
    return this.precision.refine(current, predicted);
  }
}

// =================== Select Best Target ===================
function selectBestHeadTarget(cameraPos, enemies) {
  let best = null;
  let bestDist = Infinity;
  for (const e of enemies) {
    const dx = e.boneHead.x - cameraPos.x;
    const dy = e.boneHead.y - cameraPos.y;
    const dz = e.boneHead.z - cameraPos.z;
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (d < bestDist) {
      bestDist = d;
      best = e;
    }
  }
  return best;
}
