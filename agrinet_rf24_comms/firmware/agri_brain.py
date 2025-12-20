import time
import logging
import urllib.parse
import urllib.request
from datetime import datetime
from arduino.app_utils import App, Bridge

DB_FILE     = 'agrinet_data.db'

# Blynk Configuration
BLYNK_PROTOCOL = "https" 
BLYNK_HOST     = "agrinet.io" 
BLYNK_TOKEN    = "ZyqYyNm-9B-vbne2oC23gspI_qt6tqRI"

# Logging Setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------------
# Blynk Bridge Logic
# -----------------------------------------------------------------------------
def blynk_batch_update(values):
    """Update virtual pins via Blynk HTTP API."""
    if not BLYNK_TOKEN:
        return False
    
    success = True
    # Work on a copy to avoid "dictionary changed size during iteration"
    current_items = list(values.items())
    
    for pin, val in current_items:
        try:
            # Format value to 2 decimal places if it's a float
            if isinstance(val, (float)):
                val = f"{val:.2f}"
            
            # Use formal Blynk 2.0 API structure: ?token=X&pin=VX&value=Y
            qs = {
                "token": BLYNK_TOKEN,
                "pin": pin if pin.startswith('V') else f"V{pin}",
                "value": str(val)
            }
            url = f"{BLYNK_PROTOCOL}://{BLYNK_HOST}/external/api/update?{urllib.parse.urlencode(qs)}"
            
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=5) as resp:
                if resp.status != 200:
                    logger.warning(f"Blynk {pin} warning: status {resp.status}")
                    success = False
        except Exception as e:
            logger.error(f"Blynk Update Failed for {pin}: {e}")
            success = False
    return success

def blynk_log_event(event_code, description=""):
    """Log an event to Blynk."""
    try:
        url = f"{BLYNK_PROTOCOL}://{BLYNK_HOST}/external/api/logEvent?token={BLYNK_TOKEN}&code={event_code}&description={urllib.parse.quote(description)}"
        urllib.request.urlopen(url, timeout=5)
    except Exception:
        pass

# -----------------------------------------------------------------------------
# Data Structures & Models
# -----------------------------------------------------------------------------

class BayModel:
    """Represents a single Irrigation Bay and its soil state."""
    def __init__(self, bay_id):
        self.bay_id = bay_id
        self.vwc_current = 0.0          # Volumetric Water Content (%)
        self.vwc_threshold = 30.0       # Trigger irrigation below this
        self.drainage_rate = 0.5        # % VWC loss per hour (simplified model)
        self.last_update = datetime.now()
        self.prediction_hours = 0
        self.confidence = 0

    def update_sensor(self, vwc, temp):
        """Update state from live sensor data."""
        self.vwc_current = vwc
        self.last_update = datetime.now()
        self.recalculate_prediction()

    def recalculate_prediction(self):
        """Estimate hours until VWC drops below threshold."""
        if self.vwc_current <= self.vwc_threshold:
            self.prediction_hours = 0
            self.confidence = 100
        else:
            diff = self.vwc_current - self.vwc_threshold
            # Simple linear projection: diff / rate
            if self.drainage_rate > 0:
                self.prediction_hours = diff / self.drainage_rate
                self.confidence = 80 # Placeholder logic
            else:
                self.prediction_hours = 999
                self.confidence = 50
        
        logger.info(f"Bay {self.bay_id} Prediction: Irrigate in {self.prediction_hours:.1f}h (VWC: {self.vwc_current}%)")

class AgriBrain:
    def __init__(self):
        self.running = False
        # Soil sensors are at Addr 3 and 4 per Sensor_Hub.ino
        self.bays = {3: BayModel(3), 4: BayModel(4)} 
        
        # State tracking for Blynk
        self.blynk_values = {}
        self.last_blynk_update = time.time()


    def start(self):
        print("-" * 40, flush=True)
        print(" AGRI BRAIN: INTELLIGENCE ENGINE STARTING", flush=True)
        print("-" * 40, flush=True)
        self.running = True
        
        # Register Bridge Callbacks
        print("[INFO] Registering Bridge Callbacks...", flush=True)
        Bridge.provide("wx0_data", self.on_wx0)
        Bridge.provide("wx1_data", self.on_wx1)
        Bridge.provide("soil_data", self.on_soil)
        Bridge.provide("cluster_data", self.on_cluster)
        
        # Main Loop (Logic & Commands)
        print("[INFO] Starting Main Logic Loop...", flush=True)
        while self.running:
            self.loop_logic_tick()
            time.sleep(10)

    # --- Bridge Callback Handlers ---
    def on_wx0(self, hub_id, temp, rh, press, wind, wind_dir, rain):
        self.blynk_values["V10"] = temp
        self.blynk_values["V11"] = rh
        self.blynk_values["V12"] = press
        self.blynk_values["V13"] = wind
        self.blynk_values["V14"] = int(wind_dir)
        self.blynk_values["V15"] = rain
        logger.info(f"Bridge RX: WX0 Temp={temp}")

    def on_wx1(self, hub_id, solar, lux):
        self.blynk_values["V6"] = int(solar)
        self.blynk_values["V7"] = int(lux)
        logger.info(f"Bridge RX: WX1 Solar={solar}")

    def on_soil(self, hub_id, addr, vwc, temp):
        # We shift the address by -3 to align with V30-V35
        # Addr 3 -> V30 (Top), Addr 4 -> V32 (Mid), Addr 5 -> V34 (Bot)
        soil_addr = int(addr)
        
        # Boundary check to prevent mapping to non-existent pins
        if 3 <= soil_addr <= 5:
            base_pin = 30 + ((soil_addr - 3) * 2)
            
            # Update Model for AI
            if soil_addr in self.bays:
                self.bays[soil_addr].update_sensor(vwc, temp)
            else:
                self.bays[soil_addr] = BayModel(soil_addr)
                self.bays[soil_addr].update_sensor(vwc, temp)
            
            self.blynk_values[f"V{base_pin}"] = vwc
            self.blynk_values[f"V{base_pin+1}"] = temp
            logger.info(f"Bridge RX: SOIL Addr={addr} -> Mapping to V{base_pin}")
        else:
            logger.warning(f"Bridge RX: SOIL Addr={addr} is outside expected range (3-5)")



    def on_cluster(self, cluster_id, batt, panel, pump, zones):
        self.blynk_values["V0"] = 1 # Online
        self.blynk_values["V1"] = int(batt)
        self.blynk_values["V2"] = int(panel)
        self.blynk_values["V3"] = int(pump)
        self.blynk_values["V4"] = int(zones)
        logger.info(f"Bridge RX: CLUSTER Online")


    def loop_logic_tick(self):
        """Single tick of the logic: Evaluate models and update Blynk."""
        # Heartbeat to confirm the script is alive in the Console tab
        print(f"[OK] Heartbeat {datetime.now().strftime('%H:%M:%S')} - Monitoring sensors...", flush=True)
        
        # 1. Update Blynk if we have new data
        if self.blynk_values:
            # Snap a copy using atomic pop to prevent RuntimeError
            batch = {}
            while self.blynk_values:
                k, v = self.blynk_values.popitem()
                batch[k] = v

            # Add AI Predictions to this batch (V50-V53)
            # Sensors 3, 4, 5 map to V50, V52, V54...
            for depth_id in [3, 4, 5]:
                if depth_id in self.bays:
                    bay = self.bays[depth_id]
                    base_pin = 50 + ((depth_id - 3) * 2)
                    batch[f"V{base_pin}"] = round(bay.prediction_hours, 1)
                    batch[f"V{base_pin+1}"] = bay.confidence


            if blynk_batch_update(batch):
                logger.info(f"Blynk Sync Success: {len(batch)} values")
            else:
                logger.warning("Blynk Sync Failed")


            # 2. Potential ET0 logic here in future

    def send_prediction_command(self, bay_id, hours, confidence):
        """Send command back to MCU via Bridge."""
        # We can use Bridge.notify to send data TO the STM32 as well!
        Bridge.notify("pred_cmd", float(bay_id), float(hours), float(confidence))
        logger.info(f"Bridge TX: Sent Prediction for Bay {bay_id}")


if __name__ == "__main__":
    brain = AgriBrain()
    brain.start()
