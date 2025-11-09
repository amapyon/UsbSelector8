# main.py — MicroPython (Raspberry Pi Pico)
# 8台PCを1組のUSB HID(Keyboard/Mouse)で切替
# 方針:
#  - 各ICはクラス化 (MCP23017 / TS3USB30 / PI3USB14 / 74HC137)
#  - セレクタ制御をクラス化 (Usb8Selector)
#  - PCごとに必要ピン状態をPCクラスで保持 (SW/LED/MUX選択/TS3側/HC137アドレス)
#
# 仕様:
#  - 初期は全PC切断 (TS3=OE High, PI3USB14 A/B Disable, 74HC137 ~E1 High)
#  - SWを押して離したタイミングで選択
#  - 選択PCのLEDのみ点灯 (LED=Active Low)
#
# ピン対応（論理名＝物理番号）
#  TS3USB30-OE = GP00(物理1)
#  TS3USB30-S  = GP01(物理2)
#  PI3USB14A-~EN = GP02(物理4)
#  PI3USB14A-S1  = GP03(物理5)
#  PI3USB14A-S0  = GP04(物理6)
#  PI3USB14B-~EN = GP06(物理9)
#  PI3USB14B-S1  = GP07(物理10)
#  PI3USB14B-S0  = GP08(物理11)
#  74HC137-~E1   = GP19(物理25)
#  74HC137-A2    = GP20(物理26)
#  74HC137-A1    = GP21(物理27)
#  74HC137-A0    = GP22(物理29)
#  MCP23017-SDA  = GP16(物理21) → MCP23017 物理13
#  MCP23017-SCL  = GP17(物理22) → MCP23017 物理12
#  MCP23017-INTA = GP18(物理24) → MCP23017 物理20
#
# TS3USB30の結線
#  D1: PI3USB14(B側)  / S=0でD1
#  D2: PI3USB14(A側)  / S=1でD2
#
# MCP23017の結線（LEDはActive Low）
#  GPA7:SW0, GPA6:SW1, GPA5:SW2, GPA4:SW3, GPA3:SW4, GPA2:SW5, GPA1:SW6, GPA0:SW7
#  GPB0:LED0, GPB1:LED1, GPB2:LED2, GPB3:LED3, GPB4:LED4, GPB5:LED5, GPB6:LED6, GPB7:LED7

from machine import Pin, I2C
import utime
import sys
import select

DEBUG_MODE = True # Set to False for production

# ========= MCP23017 Register Map (Bank=0) =========
IODIRA   = 0x00
IODIRB   = 0x01
IPOLA    = 0x02
IPOLB    = 0x03
GPINTENA = 0x04
GPINTENB = 0x05
DEFVALA  = 0x06
DEFVALB  = 0x07
INTCONA  = 0x08
INTCONB  = 0x09
IOCON    = 0x0A  # also 0x0B when BANK=0
GPPUA    = 0x0C
GPPUB    = 0x0D
INTFA    = 0x0E
INTFB    = 0x0F
INTCAPA  = 0x10
INTCAPB  = 0x11
GPIOA    = 0x12
GPIOB    = 0x13
OLATA    = 0x14
OLATB    = 0x15

# ================= IC Driver Classes =================

class MCP23017:
    def __init__(self, i2c: I2C, addr=0x20, inta_pin=None):
        self.i2c = i2c
        self.addr = addr
        self.inta_pin = inta_pin  # machine.Pin for INTA (active low)
        # IOCON: SEQOP=1(逐次アドレス無効), 他は既定
        self._write8(IOCON, 0b00100000)
        # A: 入力(スイッチ), プルアップ有効
        self._write8(IODIRA, 0xFF)
        self._write8(GPPUA, 0xFF)
        # B: 出力(LED Active Low) 初期は全消灯(=全1)
        self._write8(IODIRB, 0x00)
        self._write8(OLATB, 0xFF)
        # 割り込み: Aポートの変化割り込み
        self._write8(INTCONA, 0x00)   # 変化検出
        self._write8(DEFVALA, 0x00)
        self._write8(GPINTENA, 0xFF)  # 全bit有効
        # 初回読出しでラッチ解除
        self.read_gpioa()

    def _write8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read8(self, reg) -> int:
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def read_gpioa(self) -> int:
        return self._read8(GPIOA)

    def set_led_only(self, index: int):
        """
        LEDはActive Low。index(0-7)のみ点灯(=0)、他は消灯(=1)。
        index<0 のときは全消灯（=0xFF）。
        """
        mask = 0xFF if index < 0 else (0xFF & ~(1 << index))
        self._write8(OLATB, mask)

class PI3USB14:
    """ 4:1 ハイスピードUSBスイッチ (~EN: Lowで有効, S1,S0: 2bit選択0..3) """
    def __init__(self, en_n_pin: Pin, s1_pin: Pin, s0_pin: Pin, name="A"):
        self.en_n = en_n_pin; self.s1 = s1_pin; self.s0 = s0_pin
        self.name = name
        for p in (self.en_n, self.s1, self.s0):
            p.init(Pin.OUT, value=1)  # 既定: Disable
        self.disable()

    def select(self, port0123: int):
        p = max(0, min(3, port0123))
        self.s0.value((p >> 0) & 1)
        self.s1.value((p >> 1) & 1)

    def enable(self):
        self.en_n.value(0)

    def disable(self):
        self.en_n.value(1)
        
    def debug_print(self):
        print(self.name)

class TS3USB30:
    """ 2:1 ハイスピードUSBスイッチ (OE:0=Enable, S=0:D1(B側), S=1:D2(A側)) """
    def __init__(self, oe_pin: Pin, s_pin: Pin):
        self.oe = oe_pin; self.s = s_pin
        self.oe.init(Pin.OUT, value=1)  # 初期: 切断
        self.s.init(Pin.OUT, value=0)   # 既定: B側(D1)

    def route_to_A(self): self.s.value(1)
    def route_to_B(self): self.s.value(0)
    def enable(self):     self.oe.value(0)
    def disable(self):    self.oe.value(1)

class HC137:
    """ 74HC137 3→8デコーダ (~E1:Highで全OFF, A2..A0: アドレス, 選択YのみLow) """
    def __init__(self, e1_n: Pin, a2: Pin, a1: Pin, a0: Pin):
        self.e1_n = e1_n; self.a2 = a2; self.a1 = a1; self.a0 = a0
        for p in (self.e1_n, self.a2, self.a1, self.a0):
            p.init(Pin.OUT, value=1)
        self.all_off()

    def all_off(self):
        self.e1_n.value(1)  # 全出力High（=CH217K全OFF）

    def select_addr(self, addr: int):
        """ E1は変更しないで、A2..A0のみセット """
        a = max(0, min(7, addr))
        self.a0.value((a >> 0) & 1)
        self.a1.value((a >> 1) & 1)
        self.a2.value((a >> 2) & 1)

    def enable(self):  self.e1_n.value(0)
    def disable(self): self.e1_n.value(1)

# ================= PC Class =================
class PC:
    """
    各PCの“必要ピン状態”を保持するデータクラス
      - index: 0..7
      - sw_bit: MCP23017 GPAのビット位置 (SWn → GPAビット)
      - led_bit: MCP23017 GPBのビット位置 (LEDn → GPBビット, Active Low)
      - mux_side: 'A' or 'B'
      - mux_sel: 0..3 (PI3USB14のS1,S0)
      - ts3_side: 'A' or 'B' (TS3USB30のS)
      - hc_addr: 0..7 (74HC137のアドレス A2..A0、E1はLowで有効)
    """
    def __init__(self, index, sw_bit, led_bit, mux_side, mux_sel, ts3_side, hc_addr):
        self.index = index
        self.sw_bit = sw_bit
        self.led_bit = led_bit
        self.mux_side = mux_side
        self.mux_sel = mux_sel
        self.ts3_side = ts3_side
        self.hc_addr = hc_addr

    def __repr__(self):
        return ("<PC{idx}: SW=GPA{sw}, LED=GPB{led}, PI3{side}[{sel}], TS3={ts3}, HC137={hc}>"
                .format(idx=self.index, sw=self.sw_bit, led=self.led_bit,
                        side=self.mux_side, sel=self.mux_sel,
                        ts3=self.ts3_side, hc=self.hc_addr))
    
    def debug_print(self):
        print(self.__repr__())

# ================= Selector =================
class Usb8Selector:
    """
    上位制御:
      - UI: MCP23017 (GPA=SW, GPB=LED Active Low)
      - データ経路: TS3USB30 + PI3USB14(A/B)
      - VBUS: 74HC137 (→ CH217K)
      - PCテーブル: PCクラスの配列で一元管理
    """
    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        # ==== ハード初期化 ====
        self.ts3 = TS3USB30(Pin(0, Pin.OUT), Pin(1, Pin.OUT))
        self.muxA = PI3USB14(Pin(2, Pin.OUT), Pin(3, Pin.OUT), Pin(4, Pin.OUT), name="A")
        self.muxB = PI3USB14(Pin(6, Pin.OUT), Pin(7, Pin.OUT), Pin(8, Pin.OUT), name="B")
        self.hc137 = HC137(Pin(19, Pin.OUT), Pin(20, Pin.OUT), Pin(21, Pin.OUT), Pin(22, Pin.OUT))

        self.i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)
        self.inta_pin = Pin(18, Pin.IN, Pin.PULL_UP)  # INTAはActive Low
        self.io = MCP23017(self.i2c, addr=0x20, inta_pin=self.inta_pin)

        # ==== PCテーブル ====
        # SW割付: SW0..7 → GPA7..0（ご指定）
        # LED割付: LED0..7 → GPB0..7（Active Low）
        self.pcs = [
            PC(index=0, sw_bit=7, led_bit=0, mux_side='B', mux_sel=3, ts3_side='B', hc_addr=0),  # A:000
            PC(index=1, sw_bit=6, led_bit=1, mux_side='B', mux_sel=2, ts3_side='B', hc_addr=1),  # A:001
            PC(index=2, sw_bit=5, led_bit=2, mux_side='B', mux_sel=1, ts3_side='B', hc_addr=2),  # A:010
            PC(index=3, sw_bit=4, led_bit=3, mux_side='B', mux_sel=0, ts3_side='B', hc_addr=3),  # A:011
            PC(index=4, sw_bit=3, led_bit=4, mux_side='A', mux_sel=3, ts3_side='A', hc_addr=4),  # B:100
            PC(index=5, sw_bit=2, led_bit=5, mux_side='A', mux_sel=2, ts3_side='A', hc_addr=5),  # B:101
            PC(index=6, sw_bit=1, led_bit=6, mux_side='A', mux_sel=1, ts3_side='A', hc_addr=6),  # B:110
            PC(index=7, sw_bit=0, led_bit=7, mux_side='A', mux_sel=0, ts3_side='A', hc_addr=7),  # B:111
        ]
        # SWビット→PCインデックスの逆引き
        self.swbit_to_index = {pc.sw_bit: pc.index for pc in self.pcs}

        # ==== 状態 ====
        self.selected = -1  # -1: 全切断
        self.last_sw_state = self.io.read_gpioa()  # 1=未押下(プルアップ)
        self.debounce_ms = 25

        # 安全初期化: 全切断＋LED全消灯
        self._disconnect_all()
        self._update_leds()

        # 割り込み設定
        self.inta_pin.irq(handler=self._on_mcp_int, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

    # ---------- 内部: 切断／選択 ----------
    def _disconnect_all(self):
        # 経路OFF
        self.ts3.disable()
        self.muxA.disable()
        self.muxB.disable()
        # VBUS全OFF
        self.hc137.disable()  # ~E1 High
        self.selected = -1

    def _route_to_pc(self, pc: PC):
        # 1) 経路OFF
        self.ts3.disable()
        self.muxA.disable()
        self.muxB.disable()
        self.hc137.disable()  # いったん全OFF

        # 2) 事前設定（アドレス/セレクタ）
        # 74HC137のアドレスを設定（E1はまだHighのまま）
        self.hc137.select_addr(pc.hc_addr)

        # PI3USB14の選択 (どちらかのみEnable)
        if pc.mux_side == 'A':
            self.muxA.select(pc.mux_sel); self.muxA.enable()
            self.muxB.disable()
            self.ts3.route_to_A()  # TS3 S=1 → D2(A側)
        else:
            self.muxB.select(pc.mux_sel); self.muxB.enable()
            self.muxA.disable()
            self.ts3.route_to_B()  # TS3 S=0 → D1(B側)

        # 3) VBUSを有効化（選択YのみLowになる）
        self.hc137.enable()   # ~E1 Low

        # 4) 経路ON（小休止後）
        utime.sleep_ms(2)
        self.ts3.enable()

        self.selected = pc.index
        if self.debug_mode:
            pc.debug_print()

    def _update_leds(self):
        self.io.set_led_only(self.selected)

    # ---------- 公開API ----------
    def select_pc(self, idx: int):
        if not (0 <= idx < len(self.pcs)):
            self._disconnect_all()
        else:
            if idx != self.selected:
                self._route_to_pc(self.pcs[idx])
        self._update_leds()

    def disconnect(self):
        self._disconnect_all()
        self._update_leds()

    def debug_print_status(self):
        print("\n--- Usb8Selector Status ---[{}]".format(utime.ticks_ms()))
        print("Selected PC: {}".format(self.selected))
        # TS3USB30 status
        print("TS3USB30: OE={}, S={}".format(self.ts3.oe.value(), self.ts3.s.value()))
        # PI3USB14 Mux A status
        print("Mux A: ~EN={}, S1={}, S0={}".format(self.muxA.en_n.value(), self.muxA.s1.value(), self.muxA.s0.value()))
        # PI3USB14 Mux B status
        print("Mux B: ~EN={}, S1={}, S0={}".format(self.muxB.en_n.value(), self.muxB.s1.value(), self.muxB.s0.value()))
        # HC137 status
        print("HC137: ~E1={}, A2={}, A1={}, A0={}".format(self.hc137.e1_n.value(), self.hc137.a2.value(), self.hc137.a1.value(), self.hc137.a0.value()))
        # MCP23017 status
        print("MCP23017: Last SW state (GPIOA) = 0b{:08b}".format(self.last_sw_state))
        print("--------------------------")

    # ---------- 割り込み処理 ----------
    def _on_mcp_int(self, pin):
        utime.sleep_ms(self.debounce_ms)       # デバウンス
        state = self.io.read_gpioa()           # 1=未押下, 0=押下
        changed = self.last_sw_state ^ state
        if changed == 0:
            return
        # 押して→離した(0→1)を検出
        rising = (~self.last_sw_state) & state & 0xFF
        if rising:
            # 同時押し時はビット番号の小さい方(GPA0側)優先にするならrange(0,8)
            # 今回はSW0..7がGPA7..0なので、優先はSW0→SW7にしたい場合はbitを7..0で見る
            for bit in range(7, -1, -1):
                if rising & (1 << bit):
                    idx = self.swbit_to_index.get(bit, None)
                    if idx is not None:
                        self.select_pc(idx)
                        break
        self.last_sw_state = state

# ========= エントリポイント =========
def main():
    sel = Usb8Selector(DEBUG_MODE)
    print("USB HID 8台切替セレクタ：起動完了（初期は全切断・LED全消灯）。")
    print("コンソールからコマンド入力可能 (helpで一覧)")

    # 標準入力のポーリング準備
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)

    def print_help():
        print("\n--- Console Commands ---")
        print("  help          : このヘルプを表示")
        print("  status        : 現在の状態を表示")
        print("  select <0-7>  : PCを選択")
        print("  disconnect    : 全てのPCを切断")
        print("------------------------")

    try:
        last_print_time = utime.ticks_ms()
        while True:
            # コンソールからのコマンド処理 (ノンブロッキング)
            if poller.poll(0):
                cmd = sys.stdin.readline().strip()
                parts = cmd.split()
                if not parts:
                    continue

                if parts[0] == "help":
                    print_help()
                elif parts[0] == "status":
                    sel.debug_print_status()
                elif parts[0] == "disconnect":
                    sel.disconnect()
                    print("コンソールコマンド: 全切断しました。")
                elif parts[0] == "select" and len(parts) > 1:
                    try:
                        idx = int(parts[1])
                        if 0 <= idx <= 7:
                            sel.select_pc(idx)
                            print("コンソールコマンド: PC{}を選択しました。".format(idx))
                        else:
                            print("エラー: PC番号は0-7で指定してください。")
                    except ValueError:
                        print("エラー: PC番号が不正です。")
                else:
                    print("エラー: 不明なコマンド '{}'".format(cmd))

            # 5秒ごとに状態をデバッグ表示
            if utime.ticks_diff(utime.ticks_ms(), last_print_time) > 5000:
                if DEBUG_MODE:
                    sel.debug_print_status()
                last_print_time = utime.ticks_ms()
            utime.sleep_ms(100) # 少し待機

    except KeyboardInterrupt:
        sel.disconnect()
        print("終了：全切断。")

if __name__ == "__main__":
    main()
