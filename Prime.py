## Disclaimer
#verCar Prime is an independent fan project and is not affiliated 
#with, endorsed by, or connected to Anki, Inc. or Digital Dream Labs.
#All trademarks remain property of their respective owners.
#This tool is provided as-is for educational and research purposes only.
#Use at your own risk.

## Setup
#1. Flash `Firmware\Prime_Appside_fixed\Prime_Appside_fixed.ino` to ESP32 #1
#2. Flash `Firmware\Prime_Carside_fixed\Prime_Carside_fixed.ino` to ESP32 #2
#3. Install Python dependencies: `pip install customtkinter pyserial`
#4. Run `python Prime.py`


import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
import datetime
import os
import re

# --- LOOKUP ---
CAR_MODELS = {
    0x01: "Kourai (Drive)", 0x02: "Boson (Drive)", 0x03: "Rho (Drive)",
    0x04: "Katal (Drive)", 0x05: "Corax (Drive Rare)", 0x06: "Hadion (Drive Rare)",
    0x07: "Spectrix (Drive Rare)", 0x08: "Ground Shock", 0x09: "Skull",
    0x0A: "Thermo", 0x0B: "Nuke", 0x0C: "Guardian", 0x0E: "Big Bang",
    0x0F: "Freewheel (Truck)", 0x10: "X52 (Truck)", 0x11: "X52 ICE (Truck)",
    0x12: "Mammoth (MXT)", 0x13: "Dynamo (Ice Charger)", 0x14: "Nuke Phantom"
}

OVERCAR_MSG_TYPES = {
    0x0b: "Query Status/Ping", 0x0c: "Status Response", 0x16: "Version Request",
    0x17: "Ping Response", 0x19: "Version Response", 0x1b: "Battery Request",
    0x1c: "Battery Response", 0x1f: "Set SDK Track Mode",
    0x24: "Set Speed", 0x27: "Position Update", 0x2c: "Track Metadata", 0x2d: "Layout Sync",
    0x2e: "ACK", 0x30: "Track config", 0x31: "Config ACK",
    0x29: "Transition Update", 0x2b: "Vehicle Delocalized", 0x32: "Set Offset",
    0x33: "Set Lights", 0x3F: "Battery Level (OD)", 0x43: "Vehicle Status",
    0x86: "Localization State", 0x34: "Engine Light", 0x35: "Set Config",
    0x36: "Keepalive", 0x37: "Driving Style", 0x38: "Aggression",
    0x39: "Follow distance", 0x3a: "AI Control",  0x40: "Charging State",
    0x42: "Thermal Status", 0x47: "Weapon Trigger", 0x49: "Weapon Hit",
    0x4a: "Shield State",
}

# --- TRANSLATIONS ---
STRINGS = {
    "en": {
        "title":            "OverCar Prime Command Center",
        "log_label":        "📄 Log: ",
        "esp_app_search":   "🔌 ESP1 (App): Searching...",
        "esp_car_search":   "🔌 ESP2 (Car): Searching...",
        "port_placeholder": "e.g. COM3",
        "btn_scan":         "Start BLE Scan",
        "car_wait":         "Waiting for hardware...",
        "btn_connect_car":  "Connect Vehicle",
        "lbl_car_disc":     "Vehicle: Disconnected",
        "lbl_vehicle_id":   "Active Vehicle: --",
        "app_offline":      "📱 APP: OFFLINE",
        "app_online":       "📱 APP: CONNECTED",
        "tab_dash":         "Dashboard",
        "tab_vehicle":      "Test Vehicle",
        "tab_sniffer":      "Sniffer",
        "tab_expert":       "Expert",
        "card_speed":       "Speed",
        "card_offset":      "Lane Offset",
        "card_track":       "Track (Piece/Loc)",
        "card_status":      "Vehicle Status",
        "card_wait":        "Waiting...",
        "card_driving":     "DRIVING",
        "card_seam":        "SEAM",
        "model_label":      "Model",
        "btn_apply_model":  "APPLY MODEL",
        "sw_firmware":      "Firmware Override",
        "btn_clear":        "Clear",
        "auto_scroll":      "Auto-Scroll",
        "car_connected":    "Vehicle: Connected",
        "car_connecting":   "Vehicle: Connecting...",
        "car_disconnected": "Vehicle: Disconnected",
        "vehicle_prefix":   "Active Vehicle",
        "auto_vehicle_set": "Auto-set vehicle to",
        "lang_btn":         "🇩🇪 Deutsch",
    },
    "de": {
        "title":            "OverCar Prime Command Center",
        "log_label":        "📄 Log: ",
        "esp_app_search":   "🔌 ESP1 (App): Suche...",
        "esp_car_search":   "🔌 ESP2 (Car): Suche...",
        "port_placeholder": "z.B. COM3",
        "btn_scan":         "BLE Scan starten",
        "car_wait":         "Warte auf Hardware...",
        "btn_connect_car":  "Fahrzeug verbinden",
        "lbl_car_disc":     "Fahrzeug: Getrennt",
        "lbl_vehicle_id":   "Aktives Fahrzeug: --",
        "app_offline":      "📱 APP: OFFLINE",
        "app_online":       "📱 APP: VERBUNDEN",
        "tab_dash":         "Dashboard",
        "tab_vehicle":      "Test Fahrzeug",
        "tab_sniffer":      "Sniffer",
        "tab_expert":       "Experte",
        "card_speed":       "Geschwindigkeit",
        "card_offset":      "Spur-Versatz (Offset)",
        "card_track":       "Strecke (Piece/Loc)",
        "card_status":      "Fahrzeug Status",
        "card_wait":        "Warte...",
        "card_driving":     "FÄHRT",
        "card_seam":        "NAHT",
        "model_label":      "Modell",
        "btn_apply_model":  "MODELL ÜBERNEHMEN",
        "sw_firmware":      "Firmware Override",
        "btn_clear":        "Leeren",
        "auto_scroll":      "Auto-Scroll",
        "car_connected":    "Fahrzeug: Verbunden",
        "car_connecting":   "Fahrzeug: Verbinde...",
        "car_disconnected": "Fahrzeug: Getrennt",
        "vehicle_prefix":   "Aktives Fahrzeug",
        "auto_vehicle_set": "Fahrzeug auto-gesetzt:",
        "lang_btn":         "🇬🇧 English",
    },
}


class OverCarHUD(ctk.CTk):
    def __init__(self):
        super().__init__()
        # Change "de" to "en" here to make English the startup default
        self.lang = "de"
        self.geometry("1150x850")
        ctk.set_appearance_mode("dark")

        now = datetime.datetime.now()
        self.log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_filename = os.path.join(self.log_dir, now.strftime("Log-%Y-%m-%d-%H-%M.txt"))
        self.init_log_file()

        self.ser_app = None
        self.ser_car = None
        self.discovered_cars = {}
        self.is_discovering = True

        self.setup_ui()
        threading.Thread(target=self.background_discovery, daemon=True).start()

    # ------------------------------------------------------------------
    # i18n helpers
    # ------------------------------------------------------------------

    def t(self, key):
        """Return translated string for current language."""
        return STRINGS[self.lang].get(key, key)

    def toggle_language(self):
        self.lang = "en" if self.lang == "de" else "de"
        self.apply_translations()

    def apply_translations(self):
        """Update every translatable UI widget after a language switch."""
        self.title(self.t("title"))
        self.btn_lang.configure(text=self.t("lang_btn"))
        self.manual_port_entry.configure(placeholder_text=self.t("port_placeholder"))
        self.btn_scan_cars.configure(text=self.t("btn_scan"))
        self.btn_connect_car.configure(text=self.t("btn_connect_car"))
        self.btn_clear_sniffer.configure(text=self.t("btn_clear"))
        self.sw_auto_scroll.configure(text=self.t("auto_scroll"))
        self.sw_firmware.configure(text=self.t("sw_firmware"))
        self.btn_apply_model.configure(text=self.t("btn_apply_model"))
        self.lbl_model_title.configure(text=self.t("model_label"))

        # Dashboard card titles
        self.card_speed_lbl.configure(text=self.t("card_speed"))
        self.card_offset_lbl.configure(text=self.t("card_offset"))
        self.card_track_lbl.configure(text=self.t("card_track"))
        self.card_status_lbl.configure(text=self.t("card_status"))

        # ESP search labels — only if not yet connected
        if self.ser_app is None:
            self.lbl_esp_app.configure(text=self.t("esp_app_search"))
        if self.ser_car is None:
            self.lbl_esp_car.configure(text=self.t("esp_car_search"))

        # Dynamic labels: swap only if they still show a known static value
        all_disc = {v for lang in STRINGS.values() for v in [lang["lbl_car_disc"], lang["car_connected"], lang["car_disconnected"]]}
        cur_car = self.lbl_conn_car.cget("text")
        if cur_car in (STRINGS["en"]["lbl_car_disc"], STRINGS["de"]["lbl_car_disc"]):
            self.lbl_conn_car.configure(text=self.t("lbl_car_disc"))

        cur_app = self.lbl_app_status.cget("text")
        if cur_app in (STRINGS["en"]["app_offline"], STRINGS["de"]["app_offline"]):
            self.lbl_app_status.configure(text=self.t("app_offline"))
        elif cur_app in (STRINGS["en"]["app_online"], STRINGS["de"]["app_online"]):
            self.lbl_app_status.configure(text=self.t("app_online"))

        # Vehicle ID label prefix
        cur_vid = self.lbl_vehicle_id.cget("text")
        for lang in ("en", "de"):
            if cur_vid == STRINGS[lang]["lbl_vehicle_id"]:
                self.lbl_vehicle_id.configure(text=self.t("lbl_vehicle_id"))
                break
            # If it contains a real model name, just swap the prefix
            pfx_other = STRINGS["en" if lang == "de" else "de"]["vehicle_prefix"]
            if cur_vid.startswith(pfx_other + ":"):
                model_part = cur_vid[len(pfx_other)+1:]
                self.lbl_vehicle_id.configure(text=f"{self.t('vehicle_prefix')}:{model_part}")
                break

        # Dropdown placeholder when no cars found yet
        cur_vals = self.car_dropdown.cget("values")
        if cur_vals and cur_vals[0] in (STRINGS["en"]["car_wait"], STRINGS["de"]["car_wait"]):
            self.car_dropdown.configure(values=[self.t("car_wait")])

        # Card status dynamic values
        cur_cs = self.card_status.cget("text")
        for lang in ("en", "de"):
            if cur_cs == STRINGS[lang]["card_driving"]:
                self.card_status.configure(text=self.t("card_driving"))
                break
            if cur_cs == STRINGS[lang]["card_seam"]:
                self.card_status.configure(text=self.t("card_seam"))
                break
            if cur_cs == STRINGS[lang]["card_wait"]:
                self.card_status.configure(text=self.t("card_wait"))
                break

    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------

    def init_log_file(self):
        header = f"OVERCAR PRIME LOG SESSION STARTED: {datetime.datetime.now()}\n" + "="*80 + "\n"
        try:
            with open(self.log_filename, "w", encoding="utf-8") as f:
                f.write(header)
        except:
            pass

    def setup_ui(self):
        self.title(self.t("title"))
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ---- SIDEBAR ----
        self.sidebar = ctk.CTkFrame(self, width=280, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")

        ctk.CTkLabel(
            self.sidebar, text="OVERCAR PRIME",
            font=ctk.CTkFont(size=22, weight="bold")
        ).pack(pady=(20, 5))

        self.btn_lang = ctk.CTkButton(
            self.sidebar, text=self.t("lang_btn"), width=130,
            fg_color="transparent", border_width=1,
            command=self.toggle_language
        )
        self.btn_lang.pack(pady=(0, 8))

        self.lbl_log_info = ctk.CTkLabel(
            self.sidebar,
            text=f"{self.t('log_label')}{self.log_filename}",
            font=ctk.CTkFont(size=10), text_color="gray"
        )
        self.lbl_log_info.pack(fill="x", padx=20, pady=(0, 10))

        self.lbl_esp_app = ctk.CTkLabel(self.sidebar, text=self.t("esp_app_search"), text_color="orange")
        self.lbl_esp_app.pack(fill="x", padx=20)
        self.lbl_esp_car = ctk.CTkLabel(self.sidebar, text=self.t("esp_car_search"), text_color="orange")
        self.lbl_esp_car.pack(fill="x", padx=20)

        ctk.CTkLabel(self.sidebar, text="-"*30, text_color="gray").pack(pady=5)

        self.manual_port_entry = ctk.CTkEntry(self.sidebar, placeholder_text=self.t("port_placeholder"))
        self.manual_port_entry.pack(pady=5, padx=20)

        btn_m = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        btn_m.pack(pady=5)
        ctk.CTkButton(btn_m, text="APP-ESP", width=90, command=lambda: self.force_connect(1)).grid(row=0, column=0, padx=5)
        ctk.CTkButton(btn_m, text="CAR-ESP", width=90, command=lambda: self.force_connect(2)).grid(row=0, column=1, padx=5)

        ctk.CTkLabel(self.sidebar, text="-"*30, text_color="gray").pack(pady=5)

        self.btn_scan_cars = ctk.CTkButton(
            self.sidebar, text=self.t("btn_scan"),
            command=self.start_ble_scan, state="disabled"
        )
        self.btn_scan_cars.pack(pady=10, padx=20)

        self.car_dropdown = ctk.CTkComboBox(self.sidebar, values=[self.t("car_wait")], width=220)
        self.car_dropdown.pack(pady=5, padx=20)

        self.btn_connect_car = ctk.CTkButton(
            self.sidebar, text=self.t("btn_connect_car"),
            command=self.connect_to_selected_car, state="disabled"
        )
        self.btn_connect_car.pack(pady=5, padx=20)

        self.lbl_conn_car = ctk.CTkLabel(self.sidebar, text=self.t("lbl_car_disc"), anchor="w")
        self.lbl_conn_car.pack(fill="x", padx=20, pady=(20, 0))

        self.lbl_vehicle_id = ctk.CTkLabel(self.sidebar, text=self.t("lbl_vehicle_id"), anchor="w")
        self.lbl_vehicle_id.pack(fill="x", padx=20, pady=(15, 5))

        self.lbl_app_status = ctk.CTkLabel(
            self.sidebar, text=self.t("app_offline"),
            text_color="red", font=ctk.CTkFont(weight="bold")
        )
        self.lbl_app_status.pack(pady=5)

        # ---- MAIN AREA ----
        self.main_frame = ctk.CTkFrame(self, corner_radius=15)
        self.main_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")

        self.tabview = ctk.CTkTabview(self.main_frame)
        self.tabview.pack(fill="both", expand=True, padx=10, pady=10)

        for tab_key in ("tab_dash", "tab_vehicle", "tab_sniffer", "tab_expert"):
            self.tabview.add(self.t(tab_key))

        # ---- DASHBOARD TAB ----
        tab_dash = self.tabview.tab(self.t("tab_dash"))
        tab_dash.grid_columnconfigure((0, 1), weight=1)

        self.card_speed,  self.card_speed_lbl  = self.create_dash_card(tab_dash, self.t("card_speed"),  "0 mm/s",           0, 0)
        self.card_offset, self.card_offset_lbl = self.create_dash_card(tab_dash, self.t("card_offset"), "0.0 mm",           0, 1)
        self.card_track,  self.card_track_lbl  = self.create_dash_card(tab_dash, self.t("card_track"),  "N/A",              1, 0)
        self.card_status, self.card_status_lbl = self.create_dash_card(tab_dash, self.t("card_status"), self.t("card_wait"),1, 1)

        # ---- TEST VEHICLE TAB ----
        tab_v = self.tabview.tab(self.t("tab_vehicle"))
        self.lbl_model_title = ctk.CTkLabel(tab_v, text=self.t("model_label"), font=ctk.CTkFont(weight="bold"))
        self.lbl_model_title.pack(pady=10)

        self.model_dropdown = ctk.CTkComboBox(
            tab_v,
            values=[f"{hex(k).upper()}: {v}" for k, v in sorted(CAR_MODELS.items())],
            width=350
        )
        self.model_dropdown.pack(pady=5)
        self.model_dropdown.set(f"0x13: {CAR_MODELS[0x13]}")

        self.btn_apply_model = ctk.CTkButton(tab_v, text=self.t("btn_apply_model"), command=self.apply_id)
        self.btn_apply_model.pack(pady=5)

        self.sw_firmware = ctk.CTkSwitch(tab_v, text=self.t("sw_firmware"), command=self.send_version_settings)
        self.sw_firmware.pack(pady=15)

        self.fw_dropdown = ctk.CTkComboBox(
            tab_v,
            values=["5A.2E (Overdrive)", "27.25 (Drive)", "60.30 (MXT)"],
            width=300
        )
        self.fw_dropdown.pack(pady=5)
        self.fw_dropdown.set("5A.2E (Overdrive)")

        # ---- SNIFFER TAB ----
        tab_sniff = self.tabview.tab(self.t("tab_sniffer"))
        ctrl_frame = ctk.CTkFrame(tab_sniff, fg_color="transparent")
        ctrl_frame.pack(fill="x", padx=10, pady=5)

        self.btn_clear_sniffer = ctk.CTkButton(
            ctrl_frame, text=self.t("btn_clear"), width=100,
            command=lambda: self.txt_sniffer.delete("1.0", "end")
        )
        self.btn_clear_sniffer.pack(side="left", padx=5)

        self.auto_scroll = ctk.BooleanVar(value=True)
        self.sw_auto_scroll = ctk.CTkSwitch(ctrl_frame, text=self.t("auto_scroll"), variable=self.auto_scroll)
        self.sw_auto_scroll.pack(side="left", padx=20)

        self.txt_sniffer = ctk.CTkTextbox(tab_sniff, font=ctk.CTkFont(family="Consolas", size=12))
        self.txt_sniffer.pack(fill="both", expand=True, padx=10, pady=10)

        # ---- EXPERT TAB ----
        self.txt_log = ctk.CTkTextbox(
            self.tabview.tab(self.t("tab_expert")),
            font=ctk.CTkFont(family="Consolas", size=12)
        )
        self.txt_log.pack(fill="both", expand=True, padx=10, pady=10)

    def create_dash_card(self, parent, title, initial_val, row, col):
        """Returns (value_label, title_label) so both can be updated later."""
        frame = ctk.CTkFrame(parent, corner_radius=10)
        frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")
        lbl_title = ctk.CTkLabel(frame, text=title, text_color="gray", font=ctk.CTkFont(size=14))
        lbl_title.pack(pady=(15, 5))
        lbl_val = ctk.CTkLabel(frame, text=initial_val, font=ctk.CTkFont(size=28, weight="bold"))
        lbl_val.pack(pady=(0, 15))
        return lbl_val, lbl_title

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.after(0, lambda: self.txt_log.insert("end", f"[{ts}] {msg}\n") or self.txt_log.see("end"))

    def log_sniff(self, direction, payload):
        ts_full = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # 1. Säuberung: Nur Hex-Zeichen behalten, alles andere (inkl. Null-Bytes) entfernen
        clean_hex = "".join(c for c in payload if c in "0123456789ABCDEFabcdef")
        if not clean_hex:
            return

        try:
            full_bytes = [int(clean_hex[i:i+2], 16) for i in range(0, len(clean_hex), 2)]
        except Exception as e:
            self.log(f"Hex-Fehler: {str(e)} in {payload[:20]}...")
            return

        # 2. Iterative Paket-Extraktion (handelt mehrere Pakete in einem Strom)
        idx = 0
        while idx < len(full_bytes):
            packet_len = full_bytes[idx]
            total_len = packet_len + 1

            if idx + total_len > len(full_bytes):
                fragment = full_bytes[idx:]
                f_hex = " ".join(f"{b:02X}" for b in fragment)
                line = f"[{ts_full}] {direction:10} | {'Fragment':<18} | {'[Unvollständig]':<40} | {f_hex}"
                self.after(0, lambda l=line: self.txt_sniffer.insert("end", l + "\n"))
                break

            packet = full_bytes[idx:idx+total_len]
            idx += total_len

            m_id = packet[1] if len(packet) >= 2 else -1
            m_name = OVERCAR_MSG_TYPES.get(m_id, f"0x{m_id:02X}" if m_id != -1 else "??")
            info = ""

            try:
                if m_id == 0x27 and len(packet) >= 9:
                    loc_id, piece_id, offset = packet[2], packet[3], packet[4]
                    if offset > 127:
                        offset -= 256
                    speed = packet[5] + (packet[6] << 8)
                    info = f"Speed: {speed}mm/s | Piece: {piece_id} | Offset: {offset}mm"
                    self.after(0, lambda s=speed, o=offset, p=piece_id, l=loc_id: (
                        self.card_speed.configure(text=f"{s} mm/s"),
                        self.card_offset.configure(text=f"{o:.1f} mm"),
                        self.card_track.configure(text=f"P:{p} / L:{l}"),
                        self.card_status.configure(text=self.t("card_driving"), text_color="#00FF00")
                    ))
                elif m_id == 0x29 and len(packet) >= 9:
                    info = f"Transition -> Piece {packet[3]}"
                    self.after(0, lambda: self.card_status.configure(
                        text=self.t("card_seam"), text_color="cyan"))
                elif m_id == 0x24 and len(packet) >= 4:
                    info = f"Target speed: {packet[2] + (packet[3] << 8)} mm/s"
            except:
                info = "[Decode error]"

            display_payload = " ".join(f"{b:02X}" for b in packet)
            output = f"{m_name:<18} | {info:<40} | {display_payload}"
            final_line = f"[{ts_full}] {direction:10} | {output}"

            self.after(0, lambda fl=final_line: (
                self.txt_sniffer.insert("end", fl + "\n"),
                self.txt_sniffer.see("end") if self.auto_scroll.get() else None
            ))
            try:
                with open(self.log_filename, "a", encoding="utf-8") as f:
                    f.write(final_line + "\n")
            except:
                pass

    # ------------------------------------------------------------------
    # Serial / BLE
    # ------------------------------------------------------------------

    def background_discovery(self):
        while self.is_discovering:
            for p in serial.tools.list_ports.comports():
                port = p.device
                if (self.ser_app and self.ser_app.port == port) or \
                   (self.ser_car and self.ser_car.port == port):
                    continue
                try:
                    s = serial.Serial(port, 500000, timeout=0.1)
                    time.sleep(2)
                    s.write(b"IDENT?\n")
                    end = time.time() + 4
                    while time.time() < end:
                        if s.in_waiting:
                            line = s.readline().decode(errors='ignore').strip()
                            if "IDENT:OVERCAR_PRIME_APP" in line:
                                s.write(b"ACK_PRIME\n")
                                self.ser_app = s
                                self.after(0, lambda p=port: self.lbl_esp_app.configure(
                                    text=f"🔌 APP-ESP: {p}", text_color="green"))
                                threading.Thread(target=self.serial_thread, args=(s, "APP"), daemon=True).start()
                                break
                            elif "IDENT:OVERCAR_PRIME_CAR" in line:
                                s.write(b"ACK_PRIME\n")
                                self.ser_car = s
                                self.after(0, lambda p=port: self.lbl_esp_car.configure(
                                    text=f"🔌 CAR-ESP: {p}", text_color="green"))
                                self.after(0, lambda: (
                                    self.btn_scan_cars.configure(state="normal"),
                                    self.btn_connect_car.configure(state="normal")
                                ))
                                threading.Thread(target=self.serial_thread, args=(s, "CAR"), daemon=True).start()
                                break
                    if not (self.ser_app and self.ser_app.port == port) and \
                       not (self.ser_car and self.ser_car.port == port):
                        s.close()
                except:
                    pass
            if self.ser_app and self.ser_car:
                self.is_discovering = False
            time.sleep(2)

    def serial_thread(self, ser, role):
        buffer = bytearray()
        while ser.is_open:
            try:
                if ser.in_waiting:
                    buffer.extend(ser.read(ser.in_waiting))
                    while b'\n' in buffer:
                        idx = buffer.index(b'\n')
                        line = buffer[:idx].decode(errors='ignore').strip()
                        buffer = buffer[idx+1:]

                        markers = ["RX_CAR=", "RX_APP=", "DEV=", "MSG="]
                        found_markers = []
                        for m in markers:
                            for match in re.finditer(re.escape(m), line):
                                found_markers.append((match.start(), m))

                        found_markers.sort()
                        if not found_markers:
                            continue

                        for i in range(len(found_markers)):
                            start_pos, current_marker = found_markers[i]
                            end_pos = found_markers[i+1][0] if i+1 < len(found_markers) else len(line)
                            content = line[start_pos + len(current_marker):end_pos].strip()

                            if not content:
                                continue

                            if current_marker == "RX_CAR=":
                                if self.ser_app:
                                    self.ser_app.write(f"TX_APP={content}\n".encode())
                                self.log_sniff("CAR->APP", content)

                            elif current_marker == "RX_APP=":
                                if self.ser_car:
                                    self.ser_car.write(f"TX_CAR={content}\n".encode())
                                self.log_sniff("APP->CAR", content)

                            elif current_marker == "DEV=":
                                p = content.split("|")
                                if len(p) >= 2:
                                    mac = p[0]
                                    m_id = int(p[1], 16)
                                    fw = f"{p[2]}.{p[3]}" if len(p) > 3 else "?.?"
                                    self.discovered_cars[mac] = {
                                        'name': CAR_MODELS.get(m_id, f"0x{m_id:02X}"),
                                        'fw': fw,
                                        'model_id': m_id
                                    }
                                    v = [f"{info['name']} | {m}" for m, info in self.discovered_cars.items()]
                                    self.after(0, lambda val=v: self.car_dropdown.configure(values=val))

                                    # Auto-set vehicle ID when a Mammoth (0x12) is discovered
                                    if m_id == 0x12:
                                        self.after(0, lambda mid=m_id: self.auto_set_vehicle(mid))

                            elif current_marker == "MSG=":
                                self.log(f"[{role}] {content}")
                                if "Verbindung zum Auto OK" in content:
                                    self.after(0, lambda: self.lbl_conn_car.configure(
                                        text=self.t("car_connected"), text_color="cyan"))
                                elif "Fehler" in content or "verloren" in content:
                                    self.after(0, lambda: self.lbl_conn_car.configure(
                                        text=self.t("car_disconnected"), text_color="white"))

                if role == "APP" and len(buffer) >= 6 and buffer[0] == 0xAA:
                    if buffer[1] == 0x01:
                        mid, state = buffer[2], buffer[3]
                        self.after(0, lambda m=mid, s=state: self.update_telemetry(m, s))
                        buffer = buffer[6:]

            except Exception as e:
                self.log(f"Serial Error: {str(e)}")
                break
            time.sleep(0.005)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def auto_set_vehicle(self, model_id):
        """Automatically apply vehicle ID when a Mammoth (0x12) is discovered via BLE scan."""
        model_name = CAR_MODELS.get(model_id, f"0x{model_id:02X}")
        target = f"{hex(model_id).upper()}: {model_name}"
        all_values = [f"{hex(k).upper()}: {v}" for k, v in sorted(CAR_MODELS.items())]
        if target in all_values:
            self.model_dropdown.set(target)
        if self.ser_app:
            self.ser_app.write(f"ID={model_id:02X}\n".encode())
        self.lbl_vehicle_id.configure(
            text=f"{self.t('vehicle_prefix')}: {model_name}"
        )
        self.log(f"{self.t('auto_vehicle_set')} {model_name} (0x{model_id:02X})")

    def force_connect(self, esp_nr):
        port = self.manual_port_entry.get().upper()
        if not port:
            return
        try:
            s = serial.Serial(port, 921600, timeout=1.0)
            time.sleep(1.5)
            s.write(b"ACK_PRIME\n")
            if esp_nr == 1:
                self.ser_app = s
                self.lbl_esp_app.configure(text=f"🔌 APP-ESP: {port}", text_color="green")
                threading.Thread(target=self.serial_thread, args=(s, "APP"), daemon=True).start()
            else:
                self.ser_car = s
                self.lbl_esp_car.configure(text=f"🔌 CAR-ESP: {port}", text_color="green")
                self.btn_scan_cars.configure(state="normal")
                self.btn_connect_car.configure(state="normal")
                threading.Thread(target=self.serial_thread, args=(s, "CAR"), daemon=True).start()
        except:
            pass

    def start_ble_scan(self):
        if self.ser_car:
            self.ser_car.write(b"SCAN\n")

    def connect_to_selected_car(self):
        val = self.car_dropdown.get()
        if "|" in val:
            mac = val.split("|")[1].strip()
            self.ser_car.write(f"CONN={mac}\n".encode())
            car_info = self.discovered_cars.get(mac, {})
            name = car_info.get("name", "")
            prefix = self.t("car_connecting").split(":")[0]
            self.after(0, lambda: self.lbl_conn_car.configure(
                text=f"{prefix}: {name}" if name else self.t("car_connecting"),
                text_color="orange"
            ))

    def update_telemetry(self, mid, state):
        model_name = CAR_MODELS.get(mid, hex(mid))
        self.lbl_vehicle_id.configure(text=f"{self.t('vehicle_prefix')}: {model_name}")
        is_conn = bool(state & 0x80)
        self.lbl_app_status.configure(
            text=self.t("app_online") if is_conn else self.t("app_offline"),
            text_color="green" if is_conn else "red"
        )

    def apply_id(self):
        val = self.model_dropdown.get()
        hid = val.split(":")[0].strip()[2:]
        if self.ser_app:
            self.ser_app.write(f"ID={hid}\n".encode())

    def send_version_settings(self):
        if not self.ser_app:
            return
        is_on = "1" if self.sw_firmware.get() else "0"
        fw_choice = self.fw_dropdown.get()
        maj, min_val = ("5A", "2E") if "5A.2E" in fw_choice else \
                       ("27", "25") if "27.25" in fw_choice else ("60", "30")
        self.ser_app.write(f"V_SPOOF={is_on},{maj},{min_val},00\n".encode())


if __name__ == "__main__":
    OverCarHUD().mainloop()
