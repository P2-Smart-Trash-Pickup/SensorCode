import sys
import os
import serial
import struct
from dotenv import load_dotenv
import re
from datetime import datetime, timezone

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.client.exceptions import InfluxDBError

# ------------------------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------------------------
SERIAL_PORT    = os.getenv('SERIAL_PORT', 'COM6')  # default to COM6 if not set
BAUDRATE       = int(os.getenv('BAUDRATE', '115200')) 
INFLUXDB_URL   = os.getenv('INFLUXDB_URL')
INFLUXDB_TOKEN = os.getenv('INFLUXDB_TOKEN')
INFLUXDB_ORG   = os.getenv('INFLUXDB_ORG')
INFLUXDB_BUCKET= os.getenv('INFLUXDB_BUCKET')

# ------------------------------------------------------------------------------
# SANITY-CHECK INFLUXDB CREDS
# ------------------------------------------------------------------------------
if not all([INFLUXDB_TOKEN, INFLUXDB_ORG, INFLUXDB_BUCKET]):
    print("Missing InfluxDB config. Please set the token/org/bucket.")
    sys.exit(1)

try:
    client    = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
    write_api = client.write_api(write_options=SYNCHRONOUS)
    # test write to InfluxDB (Establish connection and ensure bucket exists)
    # (this will fail if the bucket does not exist)
    test_pt = Point("loratest") \
        .field("ok", True) \
        .time(datetime.now(timezone.utc), WritePrecision.NS)
    write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=test_pt)
    print(f"InfluxDB OK (bucket «{INFLUXDB_BUCKET}»)")
except InfluxDBError as e:
    print("InfluxDB error on startup:", e)
    sys.exit(1)

# ------------------------------------------------------------------------------
# PACKET PARSER (expects exactly 36 bytes)
# ------------------------------------------------------------------------------
PACKET_FMT  = '< B I I H H ' + 'H'*10 + ' B H'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

def parse_decrypted_packet(hex_str):
    hex_str = re.sub(r'[^0-9A-Fa-f]', '', hex_str)
    if len(hex_str) != PACKET_SIZE * 2:
        print(f"Skipping parse: need {PACKET_SIZE*2} hex chars, got {len(hex_str)}")
        return None

    try:
        data   = bytes.fromhex(hex_str)
        unpack = struct.unpack(PACKET_FMT, data)
        pkt = {
            'version':      unpack[0],
            'deviceId':     unpack[1],
            'packetCounter':unpack[2],
            'weight1':      unpack[3] if unpack[3]<65000 else 0,
            'weight2':      unpack[4] if unpack[4]<65000 else 0,
            'distances':    [d if d<65000 else 0 for d in unpack[5:15]],
            'flags':        unpack[15],
            'crc':          unpack[16],
            'sensor_mode':  'Ultrasonic' if (unpack[15]&1) else 'ToF'
        }
        print(f"Parsed: Dev=0x{pkt['deviceId']:08X}, "
              f"Ctr={pkt['packetCounter']}, "
              f"W1={pkt['weight1']}g W2={pkt['weight2']}g "
              f"Mode={pkt['sensor_mode']}")
        return pkt

    except Exception as e:
        print("Error unpacking packet:", e)
        return None

# ------------------------------------------------------------------------------
# PRETTY PRINT: show how the data will look in InfluxDB
# (for debugging only)
# ------------------------------------------------------------------------------
def pretty_print_point(pkt):
    print("\n—  ▶️  InfluxDB Point Preview:")
    print(f"  Measurement: loradata")
    print(f"  Tags:     device_id=0x{pkt['deviceId']:08X}, sensor_mode={pkt['sensor_mode']}")
    print("  Fields:")
    print(f"    version:        {pkt['version']}")
    print(f"    packet_counter: {pkt['packetCounter']}")
    print(f"    weight1:        {pkt['weight1']}")
    print(f"    weight2:        {pkt['weight2']}")
    print(f"    flags:          {pkt['flags']}")
    print(f"    crc:            {pkt['crc']}")
    for i, d in enumerate(pkt['distances']):
        print(f"    distance_{i}:    {d}")
    print("—  =================================================\n")

def write_to_influx(pkt):
    if not pkt:
        return

    # first show the preview of how it will look in InfluxDB:
    pretty_print_point(pkt)

    point = Point("loradata") \
        .tag("device_id", f"0x{pkt['deviceId']:08X}") \
        .tag("sensor_mode", pkt['sensor_mode']) \
        .field("version",       pkt['version']) \
        .field("packet_counter",pkt['packetCounter']) \
        .field("weight1",       pkt['weight1']) \
        .field("weight2",       pkt['weight2']) \
        .field("flags",         pkt['flags']) \
        .field("crc",           pkt['crc']) \
        .time(datetime.now(timezone.utc), WritePrecision.NS)

    for i, d in enumerate(pkt['distances']):
        point.field(f"distance_{i}", d)

    try:
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Wrote packet #{pkt['packetCounter']} to InfluxDB\n")
    except InfluxDBError as e:
        print("Failed to write to InfluxDB:", e)

# ------------------------------------------------------------------------------
# SERIAL MONITOR — only parse 36-byte decrypted packets
# ------------------------------------------------------------------------------
def monitor_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Listening on {SERIAL_PORT} @ {BAUDRATE}…")
    except Exception as e:
        print("Could not open serial port:", e)
        return

    awaiting = False
    last     = datetime.now(timezone.utc)

    try:
        while True:
            raw = ser.readline().decode('utf-8', errors='ignore')
            if not raw:
                continue
            line = raw.strip()
            print("Serial:", line)

            # did we see RX HEX in header?
            if "RX Hex" in line:
                awaiting = True
                continue

            # grab the next "(len): xxxx" line [REGEX SUCKS!]
            if awaiting:
                awaiting = False
                m = re.match(r'^\((\d+)\):\s*([0-9A-Fa-f ]+)', line)
                if not m:
                    print("unexpected format after RX Hex:", line)
                    continue

                length = int(m.group(1))
                hex_str= m.group(2).replace(' ', '')

                # **only** handle the 36-byte decrypted packets
                if length != PACKET_SIZE:
                    print(f"skipping {length}-byte dump")
                    continue

                print(f"→ Decrypted packet ({length} bytes):", hex_str)
                pkt = parse_decrypted_packet(hex_str)
                write_to_influx(pkt)
                last = datetime.now(timezone.utc)

            # keep-alive if nothing for 10 s
            if (datetime.now(timezone.utc) - last).total_seconds() > 10:
                print("No packets in 10s, still listening…")
                last = datetime.now(timezone.utc)

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        ser.close()
        client.close()
        print("Exiting cleanly")

if __name__ == "__main__":
    monitor_serial()
