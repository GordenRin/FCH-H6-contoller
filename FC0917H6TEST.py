#!/usr/bin/env python
# coding: utf-8

# # 測試機器:H6 測試日期:9/17 測試目的:1H 錯誤
# ## 代碼使用說明
# ## 測試用代碼有bug屬於正常現象
# ## self.hopper_address = 0x89 預設位置=89 需自行修改成當前設備通訊位置
# ## 測試結果為發生1H 3H錯誤都能用重連設備(break)快速解決設備錯誤問題
# ## 重連後直接使用智能退幣即可刷新錯誤狀態
# ## 建議開發方向: 
# ### 軟體端:
# ### 在執行一次智能退幣(35H) 除了用 13H 指令固定偵測機器找幣狀態 結束後用23H指令檢查最終退幣狀態
# ### 如果3次智能退幣(35H)用23H檢測後 都未找齊硬幣 用代碼報錯機器缺幣讓使用者補充硬幣 避免機器過熱
# ### 硬體端:
# ### 請充分混和硬幣後再倒入馬達內部 不讓同一種硬幣在同一層的情況發生
# ### 在設計框體時保持馬達出幣口的順暢 一定要避免造成出幣口堵塞的情況

# In[ ]:


from serial.tools import list_ports
import serial
import time
import logging
import threading
from enum import Enum

# 日誌設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('hopper_control.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)

class HopperMode(Enum):
    INTELLIGENT = "智能退幣"
    MULTI_PATH = "多航道退幣"
    STATUS_CHECK = "狀態檢查"
    AUTO_RUN = "自動運行"

class HopperController:

    def __init__(self):
        self.ser = None
        self.amount_byte_order = 'msb'
        self.hopper_address = 0x03
        self.current_mode = HopperMode.STATUS_CHECK
        self.is_running = False
        self.status_thread = None
        self.thread_lock = threading.Lock()
        self.connection_tested = False
        self.is_enabled = False
        self.device_serial = None

        # 安全閾值（可調）：若任一幣別吐出數量超過則視為異常 -> 自動 stop
        self.coin_count_threshold = 200
        # 若已支付基幣遠大於請求量的倍數，則視為異常
        self.paid_multiplier_threshold = 5

    # 查詢上一命令狀態 (0x23)
    def request_last_command_status(self):
        resp = self.send_command(0x23, [], timeout_override=1)
        if not resp or len(resp) < 5:
            return "LAST CMD STATUS 無回應或數據不足"

        header = resp[3]
        if header == 0x00:
            # resp[4] 其實是 "上一命令代碼"，不是錯誤碼
            last_cmd = resp[4]
            if last_cmd == 0x35:  # 智能退幣
                parsed = self.parse_intelligent_payout_status(resp)
                if isinstance(parsed, dict):
                    return f"上一命令 (0x35 智能退幣) 狀態:\n{parsed['text']}"
                else:
                    return f"上一命令 (0x35 智能退幣) 狀態解析失敗: {parsed}"
            else:
                return f"上一命令 0x{last_cmd:02X} 已完成 (無詳細解析)"
        elif header == 0x05:
            return "上一命令被拒絕 (NACK)"
        else:
            return f"未知回覆，Header=0x{header:02X}"

    def calculate_checksum(self, cmd_without_checksum):
        """ccTalk checksum: (0x100 - (sum(bytes) & 0xFF)) & 0xFF"""
        return (0x100 - (sum(cmd_without_checksum) & 0xFF)) & 0xFF

    def find_serial_ports(self):
        ports = list_ports.comports()
        return [(p.device, p.description) for p in ports]

    def connect(self, port_name=None):
        if port_name is None:
            ports = self.find_serial_ports()
            if not ports:
                logging.error("未找到可用串列埠")
                return False
            port_name = ports[0][0]
            logging.info(f"自動選擇端口: {port_name}")

        try:
            self.ser = serial.Serial(
                port=port_name,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2,
                write_timeout=2
            )
            logging.info(f"已連接至 {port_name}")

            ok = self.test_connection_with_diagnostics()
            if ok:
                self.get_serial_number()
                self.enable_device()
                self.start_status_monitoring()
                return True
            else:
                logging.warning("通訊測試失敗，但保持連接以便診斷")
                return True

        except Exception as e:
            logging.error(f"連接失敗: {e}")
            return False

    def enable_device(self):
        if not self.ser or not self.ser.is_open:
            logging.error("串列埠未連接")
            return False
        cmd = [self.hopper_address, 0x01, 0x01, 0xA4, 0xA5]
        chk = self.calculate_checksum(cmd)
        cmd.append(chk)
        try:
            self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
        except: pass
        try:
            logging.info(f"發送啟用指令: {bytes(cmd).hex('-').upper()}")
            self.ser.write(bytes(cmd)); self.ser.flush(); time.sleep(0.2)
            response = self.ser.read(8)
            if response and len(response) >= 4 and response[3] == 0x00:
                self.is_enabled = True
                logging.info("設備啟用成功")
                return True
            else:
                logging.warning("設備啟用失敗或無響應")
                return False
        except Exception as e:
            logging.error(f"啟用指令錯誤: {e}")
            return False

    def disable_device(self):
        if not self.ser or not self.ser.is_open:
            logging.error("串列埠未連接")
            return False
        cmd = [self.hopper_address, 0x01, 0x01, 0xA4, 0x00]
        cmd.append(self.calculate_checksum(cmd))
        try:
            self.ser.write(bytes(cmd)); self.ser.flush()
            self.is_enabled = False
            logging.info("設備已禁用")
            return True
        except Exception as e:
            logging.error(f"禁用指令錯誤: {e}")
            return False

    def get_serial_number(self):
        if not self.ser or not self.ser.is_open:
            logging.error("串列埠未連接")
            return False
        cmd = [self.hopper_address, 0x00, 0x01, 0xF2]
        cmd.append(self.calculate_checksum(cmd))
        try:
            self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
            self.ser.write(bytes(cmd)); self.ser.flush(); time.sleep(0.2)
            response = self.ser.read(16)
            if response and len(response) >= 8:
                serial_bytes = response[4:7]
                self.device_serial = bytes(serial_bytes)
                logging.info(f"設備序列號: {self.device_serial.hex('-').upper()}")
                return True
            else:
                logging.warning("獲取序列號失敗或回應不足")
                return False
        except Exception as e:
            logging.error(f"獲取序列號失敗: {e}")
            return False

    def ensure_enabled(self):
        if not self.is_enabled:
            logging.info("設備未啟用，嘗試啟用...")
            return self.enable_device()
        return True

    def send_command(self, command, data=None, timeout_override=None):
        if data is None:
            data = []
        with self.thread_lock:
            if not self.ser or not self.ser.is_open:
                logging.error("串列埠未連接")
                return None
            if command in [0x35, 0x20, 0xA7] and not self.ensure_enabled():
                logging.error("設備啟用失敗，無法發送支付指令")
                return None
            data_len = len(data)
            cmd = [self.hopper_address, data_len, 0x01, command] + list(data)
            checksum = self.calculate_checksum(cmd)
            cmd.append(checksum)
            try:
                try:
                    self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
                except: pass
                logging.info(f"發送指令: {bytes(cmd).hex('-').upper()}")
                self.ser.write(bytes(cmd)); self.ser.flush()
                original_timeout = getattr(self.ser, 'timeout', None)
                if timeout_override is not None:
                    self.ser.timeout = timeout_override
                # 等待與讀取
                time.sleep(0.05 + 0.01 * data_len)
                response = self.ser.read(256)
                if timeout_override is not None and original_timeout is not None:
                    self.ser.timeout = original_timeout
                if response:
                    logging.info(f"接收響應: {response.hex('-').upper()} (長度: {len(response)} 字節)")
                    return response
                else:
                    logging.warning(f"指令 0x{command:02X} 無響應")
                    return None
            except Exception as e:
                logging.error(f"通訊錯誤: {e}")
                return None

    def analyze_response(self, response, command):
        if not response:
            return "無響應"
        if len(response) < 5:
            return f"響應長度不足: {len(response)}字節"
        dest_addr = response[0]
        data_length = response[1]
        src_addr = response[2]
        header = response[3]
        analysis = f"目標地址: {dest_addr:02X}, 數據長度: {data_length}, 源地址: {src_addr:02X}, 頭部: {header:02X}\n"
        if command == 0xEC:
            if len(response) >= 5:
                status = response[4]
                empty = (status & 0x01) != 0
                full = (status & 0x02) != 0
                analysis += f"光電狀態: 空={empty}, 滿={full} (0x{status:02X})"
        elif command == 0xA3:
            if len(response) >= 5:
                status = response[4]
                analysis += f"測試狀態: 0x{status:02X}\n"
                analysis += self.parse_test_status(status)
        elif command == 0x13:
            if len(response) >= 5:
                analysis += self.parse_status_response(response)
        elif header == 0x00:
            analysis += "指令執行成功"
        elif header == 0x05:
            analysis += "指令被拒絕 (NACK)"
        return analysis

    def parse_test_status(self, status_byte):
        status_text = []
        if status_byte & 0x01: status_text.append("電流過高")
        if status_byte & 0x02: status_text.append("支付超時")
        if status_byte & 0x04: status_text.append("馬達反轉")
        if status_byte & 0x08: status_text.append("光電阻塞(待機)")
        if status_byte & 0x10: status_text.append("光電短路(待機)")
        if status_byte & 0x20: status_text.append("光電阻塞(支付中)")
        if status_byte & 0x40: status_text.append("硬體重置")
        if status_byte & 0x80: status_text.append("支付禁用")
        return "狀態: " + (", ".join(status_text) if status_text else "正常")

    # ---------- 重要：正確解析 Intelligent Payout (0x35) 的回應（MSB then LSB） ----------
    def parse_intelligent_payout_status(self, response):
        """
        解析智能支付回應。根據協議，回應格式:
        Reply: [01][nBytes][Add][00][Data1..DataN][Chk]
        Data1 = last command (0x35)
        Data2 = Number of base coins paid (MSB)
        Data3 = Number of base coins paid (LSB)
        Data4 = Number of base coins pending (MSB)
        Data5 = Number of base coins pending (LSB)
        Data6.. = 每種幣種已支付數量 (每種 2 bytes: MSB, LSB)
        """
        try:
            if not response or len(response) < 5:
                return "智能支付數據不完整"

            nBytes = response[1]
            expected_len = 4 + nBytes + 1  # reply has 1 + nBytes + 3 + chk = 5 + nBytes overall, check minimal
            if len(response) < expected_len:
                # 若回應長度比宣告短，視為不完整
                logging.warning("回應長度 (%d) 小於宣告的長度 (期望 %d)", len(response), expected_len)
                # 仍嘗試解析已到的部分
            # 取出 Data 範圍
            data = response[4:4 + nBytes]  # len(data) == nBytes (理論)
            if len(data) < 5:
                return "智能支付 payload 不足"

            if data[0] != 0x35:
                # 有時可能收到非 0x35 的 payload
                return f"上次命令不是 INTELLIGENT PAYOUT (Data1=0x{data[0]:02X})"

            # MSB then LSB
            paid = (data[1] << 8) + data[2]
            remain = (data[3] << 8) + data[4]

            coins = []
            offset = 5
            while offset + 1 < len(data):
                msb = data[offset]; lsb = data[offset + 1]
                coins.append((msb << 8) + lsb)
                offset += 2

            # 格式化輸出
            coin_parts = []
            for i, c in enumerate(coins):
                coin_parts.append(f"類型{i+1}: {c}枚")
            coin_str = ", ".join(coin_parts) if coin_parts else "無"

            return {
                "paid": paid,
                "remain": remain,
                "coins": coins,
                "text": f"智能支付中 - 已支付: {paid} 元, 剩餘: {remain} 元 | 使用: {coin_str}"
            }

        except Exception as e:
            logging.error("解析智能支付狀態失敗: %s", e)
            return f"解析智能支付狀態失敗: {e}"

    # 停止支付 (STOP PAYMENT 0xAC)
    def stop_payment(self):
        resp = self.send_command(0xAC, [], timeout_override=1)
        if resp and len(resp) >= 6:
            # 回應: [01][01][Add][00][Data1][Chk]
            left = resp[4]
            logging.warning(f"發出 STOP PAYMENT，Type1 剩餘未付數: {left}")
            return left
        else:
            logging.warning("STOP PAYMENT 無回應或回應不足")
            return None

    # 取消 (CANCEL 0x15)
    def cancel_current(self):
        resp = self.send_command(0x15, [], timeout_override=1)
        if resp:
            logging.info("發出 CANCEL (0x15)。回應: %s", resp.hex('-').upper())
            return resp
        else:
            logging.warning("CANCEL 無回應")
            return None

    # 在發送智能退幣後立即檢查狀態，並在必要時自動停止
    def intelligent_payout(self, amount):
        """執行智能退幣（修正：支援 MSB-first / LSB-first 發送金額）"""
        if not self.device_serial:
            logging.warning("未獲取到設備序列號，嘗試重新獲取...")
            if not self.get_serial_number():
                return "無法獲取設備序列號"

        # 基本檢查
        if amount <= 0:
            return "金額需大於 0"
        if amount > 1000000:
            return "請求金額過大，拒絕執行"

        # 拆高低位
        amount_low = amount & 0xFF
        amount_high = (amount >> 8) & 0xFF

        # 根據設定決定發送順序
        if getattr(self, 'amount_byte_order', 'msb') == 'msb':
            # 高位先 (MSB first) -> device 會把 data[0] 作為 MSB
            data = list(self.device_serial) + [amount_high, amount_low]
        elif getattr(self, 'amount_byte_order', 'msb') == 'lsb':
            # 低位先 (LSB first) -> 舊行為
            data = list(self.device_serial) + [amount_low, amount_high]
        else:
            # auto: 先使用 msb（你遇到的問題就是這個），若未來需要可改為嘗試偵測
            data = list(self.device_serial) + [amount_high, amount_low]

        # 發送智能退幣指令
        response = self.send_command(0x35, data)
        result_text = self.analyze_response(response, 0x35)

        # 立刻查一次狀態並解析
        status_resp = self.send_command(0x13, [], timeout_override=2)
        if status_resp:
            parsed = self.parse_intelligent_payout_status(status_resp)
            # parse_intelligent_payout_status 應回傳 dict (含 "paid","remain","coins","text")
            if isinstance(parsed, dict):
                logging.info(parsed["text"])
                result_text += "\n即時狀態: " + parsed["text"]
                # 若 parsed["paid"] == amount << 8（表示裝置把 bytes 當 MSB-first 解）
                # 可以在 log 中提示已檢測到 endianness 行為
                if parsed.get("paid") == (amount << 8):
                    logging.warning("注意：檢測到已支付 == amount<<8，表示裝置以 MSB-first 解讀金額（big-endian）。")
                    # 將設定固定為 MSB，避免下次再發錯（你可以註解掉或取消設定）
                    self.amount_byte_order = 'msb'
            else:
                result_text += "\n即時狀態解析失敗: " + str(parsed)
        else:
            result_text += "\n無法取得即時狀態"

        return result_text

    # 解析 13H / LAST COMMAND STATUS 等 (保留你原本的處理流程，但呼叫新版解析)
    def parse_status_response(self, response):
        """解析 13H 的回應並改用新版解析 0x35 的回應"""
        if len(response) < 5:
            return "響應數據不足"

        status_type = response[4]
        if status_type == 0x01:
            return "設備待機中"
        elif status_type == 0x02:
            if len(response) >= 6:
                error_code = response[5]
                return f"設備錯誤: {self.parse_error_code(error_code)} (0x{error_code:02X})"
        elif status_type == 0x19:  # 清空中
            return self.parse_emptying_status(response)
        elif status_type == 0x20:
            return self.parse_multi_payout_status(response)
        elif status_type == 0x35:  # 智能支付
            parsed = self.parse_intelligent_payout_status(response)
            if isinstance(parsed, dict):
                return parsed["text"]
            else:
                return str(parsed)
        return f"未知狀態類型: 0x{status_type:02X}"

    # 其餘解析函式 (parse_multi_payout_status, parse_emptying_status, parse_error_code 等)
    def parse_multi_payout_status(self, response):
        if len(response) < 13:
            return "多幣種支付數據不完整"
        try:
            data = response[4:]
            # data[0] = last cmd(0x20)
            # Data pairs: (MSB, LSB) for each paid/pending value
            # 下面範例僅解析前兩種類型，若有更多可擴充
            coin1_paid = (data[5] << 8) + data[6]
            coin1_remain = (data[7] << 8) + data[8]
            coin2_paid = (data[9] << 8) + data[10]
            coin2_remain = (data[11] << 8) + data[12]
            status = f"多幣種支付中 - 類型1: 已付{coin1_paid}, 待付{coin1_remain} | 類型2: 已付{coin2_paid}, 待付{coin2_remain}"
            return status
        except Exception as e:
            return f"解析多幣種支付失敗: {e}"

    def parse_emptying_status(self, response):
        if len(response) < 13:
            return "清空數據不完整"
        try:
            data = response[4:]
            coin1_ex = (data[1] << 8) + data[2]
            coin2_ex = (data[3] << 8) + data[4]
            coin3_ex = (data[5] << 8) + data[6] if len(data) > 6 else 0
            coin4_ex = (data[7] << 8) + data[8] if len(data) > 8 else 0
            coins = []
            if coin1_ex: coins.append(f"類型1 1元: {coin1_ex}枚")
            if coin2_ex: coins.append(f"類型2 5元: {coin2_ex}枚")
            if coin3_ex: coins.append(f"類型3 10元: {coin3_ex}枚")
            if coin4_ex: coins.append(f"類型4 50元2: {coin4_ex}枚")
            return "清空進行中 - 已提取: " + (", ".join(coins) if coins else "0枚")
        except Exception:
            return "解析清空狀態失敗"

    def parse_error_code(self, error_code):
        errors = []
        if error_code & 0x01: errors.append("硬幣出口偵測器持續啟動")
        if error_code & 0x02: errors.append("待機時硬幣出口偵測器啟動")
        if error_code & 0x04: errors.append("馬達永久性卡住")
        if error_code & 0x10: errors.append("硬體硬幣出口偵測器故障")
        if error_code & 0x20: errors.append("光電二極管故障")
        if error_code & 0x40: errors.append("編碼器光電故障")
        if error_code & 0x80: errors.append("觸發光電故障")
        return ", ".join(errors) if errors else "未知錯誤"

    def test_communication(self):
        logging.info("手動ccTalk通訊測試...")
        test_commands = [
            (0xFE, "Simple Poll"),
            (0xF6, "Request Manufacturer ID"),
            (0xF5, "Request Equipment Category ID"),
            (0xF4, "Request Product Code"),
            (0xF2, "Request Serial Number"),
            (0x13, "Request Status"),
            (0xEC, "Read Opto Status"),
            (0xA3, "Test Hopper")
        ]
        success_count = 0; device_info = []
        for cmd, name in test_commands:
            logging.info(f"測試 ccTalk 指令: {name} (0x{cmd:02X})")
            response = self.send_command(cmd, [], 3)
            if response and len(response) >= 5:
                if response[0] in (0x01,) and response[2] == self.hopper_address:
                    logging.info(f"✓ {name} 收到回應")
                    success_count += 1
            else:
                logging.warning(f"✗ {name} 失敗或無響應")
        return {'success_count': success_count, 'total': len(test_commands)}

    def test_connection_with_diagnostics(self):
        res = self.test_communication()
        if isinstance(res, dict) and res.get('success_count', 0) > 0:
            self.connection_tested = True
            logging.info(f"ccTalk 通訊測試: {res['success_count']}/{res['total']} 成功")
            return True
        else:
            self.connection_tested = False
            logging.warning("ccTalk 通訊測試全部失敗")
            return False

    def start_status_monitoring(self):
        if not self.connection_tested:
            logging.info("連接未測試通過，跳過背景監控")
            return
        if not self.is_running:
            self.is_running = True
            self.status_thread = threading.Thread(target=self._status_monitoring_loop, daemon=True)
            self.status_thread.start()
            logging.info("背景狀態監控已啟動")

    def stop_status_monitoring(self):
        if self.is_running:
            self.is_running = False
            if self.status_thread and self.status_thread.is_alive():
                self.status_thread.join(timeout=2)
            logging.info("背景狀態監控已停止")

    def _status_monitoring_loop(self):
        logging.info("開始背景狀態監控...")
        while self.is_running:
            try:
                status_response = self.send_command(0x13, [], 2)
                if status_response:
                    info = self.parse_status_response(status_response)
                    logging.info(f"[狀態監控] {info}")
                else:
                    logging.warning("[狀態監控] 無響應")
                for _ in range(30):
                    if not self.is_running: break
                    time.sleep(0.1)
            except Exception as e:
                logging.error(f"背景監控錯誤: {e}")
                time.sleep(1)
        logging.info("背景狀態監控循環已結束")

    def check_hopper_status(self):
        try:
            response = self.send_command(0x13, [], 2)
            if response:
                status_info = self.parse_status_response(response)
                opto_response = self.send_command(0xEC, [], 1)
                if opto_response and len(opto_response) >= 5:
                    opto_status = opto_response[4]
                    empty = (opto_status & 0x01) != 0
                    full = (opto_status & 0x02) != 0
                    return f"{status_info} | 光電: 空={empty}, 滿={full}"
                return status_info
            else:
                return "設備無響應"
        except Exception as e:
            logging.error(f"檢查狀態時發生錯誤: {e}")
            return f"狀態檢查錯誤: {e}"

    def multi_path_payout(self, path_number, coin_count):
        if path_number < 1 or path_number > 6:
            return "航道編號應為1-6"
        if not self.device_serial:
            return "無法獲取設備序列號"
        data = list(self.device_serial)
        for i in range(6):
            if i + 1 == path_number:
                data.extend([0x00, coin_count])
            else:
                data.extend([0x00, 0x00])
        response = self.send_command(0x20, data)
        return self.analyze_response(response, 0x20)

    def read_opto_status(self):
        response = self.send_command(0xEC)
        return self.analyze_response(response, 0xEC)

    def test_hopper(self):
        response = self.send_command(0xA3)
        return self.analyze_response(response, 0xA3)

    def device_info(self):
        info = f"設備地址: 0x{self.hopper_address:02X}\n"
        info += f"啟用狀態: {'已啟用' if self.is_enabled else '未啟用'}\n"
        if self.device_serial:
            info += f"序列號: {self.device_serial.hex('-').upper()}\n"
        else:
            info += "序列號: 未獲取\n"
        info += f"通訊測試: {'通過' if self.connection_tested else '未通過'}"
        return info

    def disconnect(self):
        self.stop_status_monitoring()
        if self.is_enabled:
            try: self.disable_device()
            except: pass
        if self.ser and self.ser.is_open:
            self.ser.close()
            logging.info("已斷開連接")


# ---------- main() 保留原本互動式介面並加入 STOP/CANCEL 選項 ----------
def main():
    controller = HopperController()
    print("=== H6 Hopper 控制程式 (含修正與安全機制) ===")
    ports = controller.find_serial_ports()
    if not ports:
        print("未找到可用串列埠"); return
    print("可用串列埠:")
    for i, (port, desc) in enumerate(ports):
        print(f"{i+1}. {port} - {desc}")
    try:
        choice = int(input("請選擇端口編號: ")) - 1
        if 0 <= choice < len(ports):
            port_name = ports[choice][0]
        else:
            print("選擇無效"); return
    except:
        print("輸入錯誤"); return

    if not controller.connect(port_name):
        return

    try:
        while True:
            print("\n=== 主選單 ===")
            print("1. 詳細通訊測試")
            print("2. 智能退幣")
            print("3. 多航道退幣")
            print("4. 檢查狀態 (13H)")
            print("5. 讀取光電狀態")
            print("6. 測試Hopper")
            print("7. 啟用設備 (A4H)")
            print("8. 禁用設備")
            print("9. 顯示設備資訊")
            print("10. 連接診斷")
            print("11. 重新連接")
            print("12. 停止支付 (STOP PAYMENT)")
            print("13. 取消 (CANCEL)")
            print("14. 退出")
            print("15. 查詢上一命令狀態 (23H)")

            status = "✓ 通訊正常" if controller.connection_tested else "✗ 通訊異常"
            status += " | 已啟用" if controller.is_enabled else " | 未啟用"

            print(f"狀態: {status}")

            choice = input("請選擇操作: ")

            if choice == "1":
                result = controller.test_communication()
                print(f"測試結果: {result}")

            elif choice == "2":
                try:
                    amount = int(input("請輸入退幣金額 (元): "))
                    result = controller.intelligent_payout(amount)
                    print(f"智能退幣結果:\n{result}")
                except Exception as e:
                    print("金額輸入錯誤:", e)

            elif choice == "3":
                try:
                    path = int(input("請選擇航道 (1-6): "))
                    count = int(input("請輸入硬幣數量: "))
                    result = controller.multi_path_payout(path, count)
                    print(f"多航道退幣結果:\n{result}")
                except:
                    print("輸入錯誤")

            elif choice == "4":
                result = controller.check_hopper_status()
                print(f"狀態檢查結果 (13H):\n{result}")

            elif choice == "5":
                result = controller.read_opto_status()
                print(f"光電狀態:\n{result}")

            elif choice == "6":
                result = controller.test_hopper()
                print(f"Hopper測試結果:\n{result}")

            elif choice == "7":
                if controller.enable_device(): print("設備啟用成功")
                else: print("設備啟用失敗")

            elif choice == "8":
                if controller.disable_device(): print("設備已禁用")
                else: print("設備禁用失敗")

            elif choice == "9":
                print(controller.device_info())

            elif choice == "10":
                ok = controller.test_connection_with_diagnostics()
                print(f"連接診斷: {'通過' if ok else '失敗'}")

            elif choice == "11":
                print("重新連接...")
                controller.disconnect(); time.sleep(1)
                if controller.connect(port_name): print("重新連接成功")
                else: print("重新連接失敗")

            elif choice == "12":
                print("發送 STOP PAYMENT...")
                left = controller.stop_payment()
                print(f"STOP PAYMENT 回應: 剩餘未付 (type1) = {left}")

            elif choice == "13":
                print("發送 CANCEL...")
                controller.cancel_current()
                print("已發送 CANCEL")

            elif choice == "14":
                break

            elif choice == "15":
                result = controller.request_last_command_status()
                print(f"上一命令狀態 (23H):\n{result}")

            else:
                print("選擇無效")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n程式被用戶中斷")
    except Exception as e:
        logging.error(f"程式錯誤: {e}")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()


# 測試版代碼

# In[8]:


get_ipython().system('jupyter nbconvert --to script FC0917H6TEST.ipynb')


# In[ ]:




