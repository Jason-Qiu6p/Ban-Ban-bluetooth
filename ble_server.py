# 文件名: ble_server.py
import ubluetooth
from micropython import const
import padog

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

# 微信小程序通用的 NUS UUID
_UART_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_NOTIFY,)
_UART_RX = (ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_WRITE | ubluetooth.FLAG_WRITE_NO_RESPONSE,)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX),)

class BLEServer:
    def __init__(self, name="BanBan-Dog"):
        self._ble = ubluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()
        self._payload = name
        self._advertise()
        print(f"蓝牙服务已启动: {name}")

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("蓝牙已连接")
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            self._advertise()
            print("蓝牙已断开")
            # 蓝牙断开时，立即停止运动
            padog.move(0, 0, 0)
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._handle_rx:
                recv_data = self._ble.gatts_read(self._handle_rx)
                # === 核心：调用 padog 的解析函数 ===
                padog.parse_command(recv_data, source="BLE")

    def _advertise(self, interval_us=500000):
        _APPEND = lambda b, d: b + bytearray((len(d) + 1, 0x09)) + d
        payload = bytearray((0x02, 0x01, 0x06))
        payload = _APPEND(payload, self._payload)
        self._ble.gap_advertise(interval_us, adv_data=payload)
