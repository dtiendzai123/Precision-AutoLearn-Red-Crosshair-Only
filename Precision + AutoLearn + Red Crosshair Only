class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  length() { return Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2); }
  normalize() {
    const len = this.length();
    return len > 0 ? this.multiplyScalar(1 / len) : new Vector3();
  }
  clone() { return new Vector3(this.x, this.y, this.z); }
  static zero() { return new Vector3(0, 0, 0); }
}

class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; this.Q = Q; this.A = 1; this.C = 1;
    this.cov = NaN; this.x = NaN;
  }

  filter(z) {
    if (isNaN(this.x)) {
      this.x = z; this.cov = this.R;
    } else {
      const predX = this.A * this.x;
      const predCov = this.cov + this.Q;
      const K = predCov * this.C / (this.C * predCov * this.C + this.R);
      this.x = predX + K * (z - this.C * predX);
      this.cov = predCov - K * this.C * predCov;
    }
    return this.x;
  }

  reset() { this.cov = NaN; this.x = NaN; }
}

// === Precision Aim ===
class PrecisionAimHelper {
  constructor() {
    this.centerBias = 0.95;
    this.kalmanX = new KalmanFilter(0.01, 0.00001);
    this.kalmanY = new KalmanFilter(0.01, 0.00001);
    this.kalmanZ = new KalmanFilter(0.01, 0.00001);
  }

  centerRefine(currentAim, headCenter) {
    const dx = this.kalmanX.filter(headCenter.x - currentAim.x);
    const dy = this.kalmanY.filter(headCenter.y - currentAim.y);
    const dz = this.kalmanZ.filter(headCenter.z - currentAim.z);
    const correction = new Vector3(dx, dy, dz).multiplyScalar(this.centerBias);
    return currentAim.add(correction);
  }
}

// === Crosshair Red Detector (Giả lập hoặc thay thế API) ===
function isCrosshairRed() {
  // Nếu bạn có API thực tế thì thay thế tại đây:
  // return GameAPI.crosshairState === "red";
  return Math.random() < 0.8; // giả lập: 80% chance đang đỏ
}

// === Auto-Learn Bone Head System ===
class AimLockSystem {
  constructor() {
    this.prevHead = null;
    this.velocity = Vector3.zero();
    this.recoilOffset = Vector3.zero();
    this.lastUpdate = Date.now();
    this.kalman = {
      x: new KalmanFilter(), y: new KalmanFilter(), z: new KalmanFilter()
    };
    this.preciseAim = new PrecisionAimHelper();
  }

  learnHead(current) {
    const now = Date.now();
    const dt = (now - this.lastUpdate) / 1000;
    if (this.prevHead && dt > 0) {
      this.velocity = current.subtract(this.prevHead).multiplyScalar(1 / dt);
    }
    this.prevHead = current.clone();
    this.lastUpdate = now;

    return new Vector3(
      this.kalman.x.filter(current.x),
      this.kalman.y.filter(current.y),
      this.kalman.z.filter(current.z)
    );
  }

  applyRecoilComp(offset) {
    this.recoilOffset = this.recoilOffset.multiplyScalar(0.9).add(offset.multiplyScalar(0.1));
  }

  setCrosshair(vec3) {
    console.log("🎯 Final Aim (HeadLock):", vec3.x.toFixed(6), vec3.y.toFixed(6), vec3.z.toFixed(6));
    // GameAPI.setAim(vec3.x, vec3.y, vec3.z);
  }

  update(currentHead, recoilOffset = Vector3.zero()) {
    if (!isCrosshairRed()) return;

    const learned = this.learnHead(currentHead);
    this.applyRecoilComp(recoilOffset);

    const rawAim = learned.subtract(this.recoilOffset);
    const refinedAim = this.preciseAim.centerRefine(this.prevHead || Vector3.zero(), rawAim);

    this.setCrosshair(refinedAim);
  }
}

// === Simulate Runtime ===
const aimSystem = new AimLockSystem();

const bone_Head = new Vector3(-0.0456970781, -0.004478302, -0.0200432576);
const recoil = new Vector3(0.005, -0.002, 0.001); // tuỳ loại vũ khí

function runLoop() {
  aimSystem.update(bone_Head, recoil);
  setTimeout(runLoop, 16); // ~60 FPS
}

runLoop();
