import serial
import json
import time
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# ── Configurações ─────────────────────────────────────────────────────────────
SERIAL_PORT   = "COM4"
BAUD_RATE     = 115200

INFLUX_URL    = "http://localhost:8086"
INFLUX_TOKEN  = "1omGd53461URznOA1byIRwTUSl3z4_Uw8cpxSixvzF11I_ZFbqOlcVEFCnA6VYwqwpbW65cRZCxUy2aWyTSTuw=="
INFLUX_ORG    = "labyrinth"
INFLUX_BUCKET = "esp32"
# ─────────────────────────────────────────────────────────────────────────────


def connect_serial():
    """Tenta conectar na serial, fica tentando até conseguir."""
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
            print(f"[SERIAL] Conectado em {SERIAL_PORT} @ {BAUD_RATE} baud")
            return ser
        except Exception as e:
            print(f"[SERIAL] Falha ao conectar: {e}. Tentando de novo em 3s...")
            time.sleep(3)


def parse_line(line: str):
    """
    Extrai o JSON da linha recebida.
    O ESP32 pode mandar linhas de log antes do JSON — ignora tudo que não for JSON.
    Formato esperado:
      {"joy":{"x":1823,"y":2047},"mpu":{"ang_x":-3.21,"ang_y":12.45},"win":0}
    """
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None


def build_point(data: dict) -> Point:
    """Converte o dicionário recebido num Point do InfluxDB."""
    point = Point("labyrinth")

    # Joystick
    joy = data.get("joy", {})
    point.field("joy_x", int(joy.get("x", 0)))
    point.field("joy_y", int(joy.get("y", 0)))

    # MPU6050
    mpu = data.get("mpu", {})
    point.field("ang_x", float(mpu.get("ang_x", 0.0)))
    point.field("ang_y", float(mpu.get("ang_y", 0.0)))

    # Estado do jogo
    point.field("win", int(data.get("win", 0)))

    return point


def main():
    # Conecta no InfluxDB
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    write_api = client.write_api(write_options=SYNCHRONOUS)
    print(f"[INFLUX] Conectado em {INFLUX_URL} → bucket '{INFLUX_BUCKET}'")

    # Conecta na serial
    ser = connect_serial()

    print("[MAIN] Lendo dados do ESP32... (Ctrl+C para parar)\n")

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore")
            data = parse_line(raw)

            if data is None:
                # Linha de log do ESP32 — imprime pra debug mas não envia
                if raw.strip():
                    print(f"[LOG ESP32] {raw.strip()}")
                continue

            point = build_point(data)
            write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)

            print(
                f"joy=({data['joy']['x']},{data['joy']['y']}) "
                f"ang=({data['mpu']['ang_x']:.2f},{data['mpu']['ang_y']:.2f}) "
                f"win={data['win']}"
            )

        except serial.SerialException as e:
            print(f"[SERIAL] Conexão perdida: {e}. Reconectando...")
            ser.close()
            ser = connect_serial()

        except KeyboardInterrupt:
            print("\n[MAIN] Encerrando...")
            break

        except Exception as e:
            print(f"[ERRO] {e}")

    ser.close()
    client.close()
    print("[MAIN] Encerrado.")


if __name__ == "__main__":
    main()
