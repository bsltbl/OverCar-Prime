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
import struct
import csv
import math
import tkinter as tk
from tkinter import Canvas

# =============================================================================
# PROTOCOL CONSTANTS  (nach MasterAirscrach Protocol.cs)
# =============================================================================

MSG = {
    "SEND_BT_DISCONNECT": 13,
    "SEND_PING": 22, "RECV_PING": 23,
    "SEND_REQUEST_CAR_VERSION": 24, "RECV_CAR_VERSION": 25,
    "SEND_BATTERY_VOLTAGE": 26, "RECV_BATTERY_VOLTAGE": 27,
    "SEND_SET_CAR_SPEED": 36, "SEND_LANE_CHANGE": 37, "SEND_CANCEL_LANE_CHANGE": 38,
    "RECV_TR_POSITION_UPDATE": 39, "RECV_TR_TRANSITION_UPDATE": 41,
    "RECV_TR_INTERSECTION_POSITION_UPDATE": 42, "RECV_TR_CAR_DELOCALIZED": 43,
    "SEND_SET_OFFSET_FROM_ROAD_CENTER": 44, "RECV_OFFSET_FROM_ROAD_CENTER_UPDATE": 45,
    "SEND_OPEN_LOOP_TURN": 50, "SEND_SET_LIGHTS_PATTERN": 51,
    "SEND_SET_OFFSET_FROM_ROAD_CENTER_ADJUSTMENT": 52,
    "RECV_SPEED_UPDATE": 54, "RECV_STATUS_UPDATE": 63,
    "RECV_LANE_CHANGE_UPDATE": 65, "RECV_TR_DELOC_AUTO_RECOVERY_ENTERED": 67,
    "RECV_TR_DELOC_AUTO_RECOVERY_SUCCESS": 68, "SEND_TR_STOP_ON_TRANSITION": 74,
    "RECV_TR_JUMP_PIECE_BOOST": 75, "RECV_TR_COLLISION_DETECTED": 77,
    "RECV_JUMP_PIECE_RESULT": 78, "RECV_SUPERCODE": 79, "RECV_TR_SPECIAL": 83,
    "RECV_DEV_CYCLE_OVERTIME": 134, "SEND_SDK_MODE": 144,
    "SEND_DEV_UPDATE_MODEL": 147, "RECV_DEV_UPDATE_MODEL_RESPONSE": 148,
}

MSG_SHORT = {
    13:"BT Disconnect", 22:"Ping →", 23:"← Ping", 24:"Version Req →", 25:"← Car Version",
    26:"Battery Req →", 27:"← Battery", 36:"Set Speed →", 37:"Lane Change →",
    38:"Cancel LC →", 39:"← Pos Update", 41:"← Transition", 42:"← Intersection",
    43:"← Delocalized", 44:"Set Offset →", 45:"← Offset Update", 50:"Open Loop Turn →",
    51:"Set Lights →", 52:"Set Offset Adj →", 54:"← Speed Update", 63:"← Status Update",
    65:"← Lane Change", 67:"← Deloc Recovery↑", 68:"← Deloc Recovery✓",
    74:"Stop on Trans →", 75:"← Jump Boost", 77:"← Collision", 78:"← Jump Result",
    79:"← Supercode", 83:"← TR Special", 134:"← Cycle Overtime",
    144:"SDK Mode →", 147:"Update Model →", 148:"← Model Response",
}

CAR_MODELS = {
    0x01:"Kourai (Drive)", 0x02:"Boson (Drive)", 0x03:"Rho (Drive)",
    0x04:"Katal (Drive)", 0x05:"Corax (Drive Rare)", 0x06:"Hadion (Drive Rare)",
    0x07:"Spectrix (Drive Rare)", 0x08:"Ground Shock", 0x09:"Skull",
    0x0A:"Thermo", 0x0B:"Nuke", 0x0C:"Guardian", 0x0E:"Big Bang",
    0x0F:"Freewheel (Truck)", 0x10:"X52 (Truck)", 0x11:"X52 ICE (Truck)",
    0x12:"Mammoth (MXT)", 0x13:"Dynamo (Ice Charger)", 0x14:"Nuke Phantom",
}

# =============================================================================
# PROTOCOL PARSER
# =============================================================================

def _s16(d, o): return struct.unpack_from('<h', bytes(d[o:o+2]))[0] if o+1 < len(d) else 0
def _u16(d, o): return struct.unpack_from('<H', bytes(d[o:o+2]))[0] if o+1 < len(d) else 0
def _f32(d, o): return struct.unpack_from('<f', bytes(d[o:o+4]))[0] if o+3 < len(d) else 0.0
def _u32(d, o): return struct.unpack_from('<I', bytes(d[o:o+4]))[0] if o+3 < len(d) else 0

def decode_packet_info(pkt):
    if len(pkt) < 2: return "?", None
    mid = pkt[1]
    info, data = "", None

    if mid == MSG["RECV_SPEED_UPDATE"] and len(pkt) >= 8:
        d = {"desiredSpeedMMPS": _s16(pkt,2), "accelMMPS2": _s16(pkt,4), "actualSpeedMMPS": _s16(pkt,6)}
        info = f"Desired: {d['desiredSpeedMMPS']} mm/s | Actual: {d['actualSpeedMMPS']} mm/s | Accel: {d['accelMMPS2']} mm/s²"
        data = ("speed", d)

    elif mid == MSG["RECV_TR_POSITION_UPDATE"] and len(pkt) >= 17:
        d = {"locationID": pkt[2], "segmentID": pkt[3], "offset": _f32(pkt,4),
             "speedMMPS": _s16(pkt,8), "parsingFlags": pkt[10],
             "lastRecvdLaneChangeCmdID": pkt[11], "lastExecutedLaneChangeCmdID": pkt[12],
             "lastDesiredLaneChangeSpeedMMPS": _u16(pkt,13), "lastDesiredSpeedMMPS": _s16(pkt,15)}
        info = f"Piece: {d['segmentID']} Loc: {d['locationID']} | Offset: {d['offset']:.1f} mm | Speed: {d['speedMMPS']} mm/s"
        data = ("position", d)

    elif mid == MSG["RECV_TR_TRANSITION_UPDATE"] and len(pkt) >= 18:
        d = {"currentSegmentIdx": pkt[2], "previousSegmentIdx": pkt[3], "offset": _f32(pkt,4),
             "lastRecvdLaneChangeCmdID": pkt[8], "lastExecutedLaneChangeCmdID": pkt[9],
             "lastDesiredLaneChangeSpeedMMPS": _u16(pkt,10), "avgFollowLineDrift": pkt[12],
             "hadLaneChangeActivity": pkt[13], "uphillCounter": pkt[14],
             "downhillCounter": pkt[15], "leftWheelDistanceCM": pkt[16], "rightWheelDistanceCM": pkt[17]}
        info = f"Seg: {d['currentSegmentIdx']} ← {d['previousSegmentIdx']} | Offset: {d['offset']:.1f} mm"
        data = ("transition", d)

    elif mid == MSG["RECV_OFFSET_FROM_ROAD_CENTER_UPDATE"] and len(pkt) >= 7:
        d = {"offsetFromCenterMM": _f32(pkt,2), "laneChangeID": pkt[6]}
        info = f"Offset: {d['offsetFromCenterMM']:.2f} mm | LC-ID: {d['laneChangeID']}"
        data = ("offset", d)

    elif mid == MSG["RECV_BATTERY_VOLTAGE"] and len(pkt) >= 4:
        mv = _u16(pkt, 2)
        info = f"{mv} mV ({mv/1000:.3f} V)"
        data = ("battery", mv)

    elif mid == MSG["RECV_STATUS_UPDATE"] and len(pkt) >= 6:
        d = {"onTrack": bool(pkt[2]&1), "onCharger": bool(pkt[3]&1),
             "hasLowBattery": bool(pkt[4]&1), "hasChargedBattery": bool(pkt[5]&1)}
        flags = [k for k,v in [("OnTrack",d["onTrack"]),("Charging",d["onCharger"]),
                                ("LowBatt!",d["hasLowBattery"]),("FullBatt",d["hasChargedBattery"])] if v]
        info = " | ".join(flags) if flags else "Idle"
        data = ("status", d)

    elif mid == MSG["RECV_LANE_CHANGE_UPDATE"] and len(pkt) >= 15:
        d = {"currentOffset": _f32(pkt,2), "targetOffset": _f32(pkt,6),
             "horizontalSpeedMMPS": _u16(pkt,10), "verticalSpeedMMPS": _s16(pkt,12), "laneChangeID": pkt[14]}
        info = f"Cur: {d['currentOffset']:.1f}→Tgt: {d['targetOffset']:.1f} mm | LC-ID: {d['laneChangeID']}"
        data = ("lane_change", d)

    elif mid == MSG["RECV_TR_INTERSECTION_POSITION_UPDATE"] and len(pkt) >= 13:
        d = {"currRoadPieceIdx": pkt[2], "offset": _f32(pkt,3), "intersectionCode": pkt[7],
             "isExiting": pkt[8]!=0, "mmSinceLastTransBar": _u16(pkt,9), "mmSinceLastIntersectionCode": _u16(pkt,11)}
        info = f"Piece: {d['currRoadPieceIdx']} | {'Exiting' if d['isExiting'] else 'Entering'} | Code: {d['intersectionCode']}"
        data = ("intersection", d)

    elif mid == MSG["RECV_TR_CAR_DELOCALIZED"]:
        info = "⚠ CAR DELOCALIZED"; data = ("delocalized", {})
    elif mid == MSG["RECV_TR_DELOC_AUTO_RECOVERY_ENTERED"]:
        info = "🔄 Auto-Recovery started"; data = ("recovery_start", {})
    elif mid == MSG["RECV_TR_DELOC_AUTO_RECOVERY_SUCCESS"]:
        info = "✅ Auto-Recovery success"; data = ("recovery_ok", {})

    elif mid == MSG["RECV_TR_COLLISION_DETECTED"] and len(pkt) >= 4:
        d = {"wasSideOnCollision": pkt[2]!=0, "wasFrontBackCollision": pkt[3]!=0}
        parts = ([("Side",d["wasSideOnCollision"]),("Front/Back",d["wasFrontBackCollision"])])
        info = "💥 Collision: " + ", ".join(k for k,v in parts if v)
        data = ("collision", d)

    elif mid == MSG["RECV_CAR_VERSION"] and len(pkt) >= 6:
        v = _u32(pkt, 2)
        info = f"Version: 0x{v:08X}"; data = ("version", v)

    elif mid == MSG["RECV_PING"]:
        info = "Pong ←"
    elif mid == MSG["SEND_SET_CAR_SPEED"] and len(pkt) >= 6:
        info = f"→ Speed: {_s16(pkt,2)} mm/s | Accel: {_s16(pkt,4)} mm/s²"
    elif mid == MSG["SEND_SDK_MODE"]:
        info = "→ SDK Mode enabled"

    return info, data

# =============================================================================
# i18n
# =============================================================================

STRINGS = {
    "en": {
        "title":"OverCar Prime — Test Dashboard",
        "log_label":"📄 Log: ", "esp_app_search":"🔌 ESP1 (App): Searching...",
        "esp_car_search":"🔌 ESP2 (Car): Searching...", "port_placeholder":"e.g. COM3",
        "btn_scan":"Start BLE Scan", "car_wait":"Waiting for hardware...",
        "btn_connect_car":"Connect Vehicle", "lbl_car_disc":"Vehicle: Disconnected",
        "lbl_vehicle_id":"Active Vehicle: --", "app_offline":"📱 APP: OFFLINE",
        "app_online":"📱 APP: CONNECTED", "tab_dash":"Dashboard", "tab_vehicle":"Test Vehicle",
        "tab_sniffer":"Sniffer", "tab_expert":"Expert", "tab_speed_graph":"Speed Graph",
        "tab_battery":"Battery", "tab_laps":"Lap Timer", "tab_velocity":"Velocity",
        "tab_about":"About",
        "card_speed_des":"Desired Speed", "card_speed_act":"Actual Speed",
        "card_battery":"Battery", "card_offset":"Lane Offset",
        "card_track":"Track Piece / Loc", "card_status":"Vehicle Status",
        "card_lane_id":"Lane Change ID", "card_wait":"Waiting...", "card_na":"N/A",
        "card_driving":"DRIVING", "card_seam":"SEAM", "card_deloc":"⚠ DELOCALIZED",
        "card_collision":"💥 COLLISION", "card_recovery":"🔄 RECOVERY",
        "model_label":"Model", "btn_apply_model":"APPLY MODEL",
        "sw_firmware":"Firmware Override", "btn_clear":"Clear",
        "auto_scroll":"Auto-Scroll", "car_connected":"Vehicle: Connected",
        "car_connecting":"Vehicle: Connecting...", "car_disconnected":"Vehicle: Disconnected",
        "vehicle_prefix":"Active Vehicle", "auto_vehicle_set":"Auto-set vehicle to",
        "lang_btn":"🇩🇪 Deutsch",
        # Sidebar
        "sidebar_subtitle":"BLE Debug Dashboard",
        # Speed graph tab
        "sg_window_label":"Window:",
        "sg_1min":"1 min", "sg_2min":"2 min", "sg_5min":"5 min", "sg_full":"Full",
        "sg_des_lbl":"Desired (mm/s)", "sg_act_lbl":"Actual (mm/s)",
        # Battery tab
        "batt_charge_indicator":"Charge Indicator (3.3 V – 4.2 V)",
        "batt_voltage_graph":"Voltage over time (charge events = gold markers)",
        "batt_voltage_card":"Voltage", "batt_status_card":"Status",
        "batt_session_card":"Session Time",
        "batt_charging":"Charging 🔌", "batt_discharging":"Discharging", "batt_idle":"Idle",
        "batt_unknown":"Unknown", "batt_volt_lbl":"Voltage (mV)",
        # Lap tab
        "lap_piece_label":"Start/Finish Piece ID:", "placeholder_auto":"auto",
        "lap_btn_set":"Set", "lap_btn_reset":"Reset Laps",
        "lap_current_label":"Current:", "lap_autodetect":"Start/Finish: [auto-detect]",
        "lap_autodetected":"Start/Finish: Piece {} (auto)",
        "lap_piece_set":"Start/Finish: Piece {}",
        "lap_best_times":"🏆 Best Times",
        "lap_saved_hint":"💾 Saved to times_*.csv",
        "lap_reset_timer":"⏱ --.-s",
        "lap_table_title":"Last 15 Laps", "lap_prefix":"Lap ",
        "lap_col_num":"#", "lap_col_time":"Time", "lap_col_gap":"Gap", "lap_col_start":"Start",
        # Velocity tab
        "vel_speed_title":"Speed Over Time",
        "vel_offset_title":"Lane Offset (Left ← 0 → Right)",
        "vel_act_lbl":"Actual Speed (mm/s)", "vel_off_lbl":"Offset (mm)", "vel_tgt_lbl":"Target Offset (mm)",
        # Vehicle tab
        "veh_speed_label":"Speed mm/s:", "veh_accel_label":"Accel mm/s²:",
        "veh_send_speed":"Send Speed →", "veh_quick_cmds":"Quick Commands",
        "veh_ping":"Ping →", "veh_battery_req":"Battery? →",
        "veh_sdk_mode":"SDK Mode →", "veh_version_req":"Version? →",
        # About tab
        "about_version":"v2.9  ·  BLE Man-in-the-Middle Debug Tool",
        "about_desc":"A fan-made Man-in-the-Middle BLE debug tool for Anki Overdrive,\nbuilt with two ESP32 modules and a Python HUD.",
        "about_feat1":"🔵  Intercepts BLE traffic between the Overdrive app and vehicles",
        "about_feat2":"📡  Decodes and displays packets in real time (Sniffer)",
        "about_feat3":"⚙️   Allows vehicle ID and firmware changing for research purposes",
        "about_feat4":"📊  Live speed, battery and offset graphs",
        "about_feat5":"🏁  Lap timer with auto-detection of start/finish line",
        "about_thanks":"Thanks to MasterAirscrach — his Protocol.cs is the best 🙏",
        "about_disclaimer_title":"⚠  Disclaimer",
        "about_disclaimer":"OverCar Prime is an independent fan project and is not affiliated with,\nendorsed by, or connected to Anki, Inc. or Digital Dream Labs.\nAll trademarks remain property of their respective owners.\nThis tool is provided as-is for educational and research purposes only.\nUse at your own risk.",
        "about_tech_title":"Tech Stack",
        "about_tech1":"Python 3  ·  customtkinter  ·  pyserial",
        "about_tech2":"ESP32 (×2)  ·  Arduino BLE libraries",
        "about_tech3":"Protocol: MasterAirscrach / OverdriveServer",
        # Status / Details
        "status_recovered":"✅ Recovered",
        "status_track":" (Track)", "status_low_batt":"Low Batt! 🔴", "status_charged":"Charged ✅",
        # Sniffer
        "sniff_time":"[Time]", "sniff_dir":"Dir", "sniff_msg":"Message", "sniff_info":"Info", "sniff_bytes":"Bytes"
    },
    "de": {
        "title":"OverCar Prime — Test Dashboard",
        "log_label":"📄 Log: ", "esp_app_search":"🔌 ESP1 (App): Suche...",
        "esp_car_search":"🔌 ESP2 (Car): Suche...", "port_placeholder":"z.B. COM3",
        "btn_scan":"BLE Scan starten", "car_wait":"Warte auf Hardware...",
        "btn_connect_car":"Fahrzeug verbinden", "lbl_car_disc":"Fahrzeug: Getrennt",
        "lbl_vehicle_id":"Aktives Fahrzeug: --", "app_offline":"📱 APP: OFFLINE",
        "app_online":"📱 APP: VERBUNDEN", "tab_dash":"Dashboard", "tab_vehicle":"Test Fahrzeug",
        "tab_sniffer":"Sniffer", "tab_expert":"Experte", "tab_speed_graph":"Geschwindigkeit",
        "tab_battery":"Batterie", "tab_laps":"Rundentimer", "tab_velocity":"Versatz",
        "tab_about":"Info",
        "card_speed_des":"Soll-Geschwindigkeit", "card_speed_act":"Ist-Geschwindigkeit",
        "card_battery":"Batterie", "card_offset":"Spur-Versatz",
        "card_track":"Streckensegment / Loc", "card_status":"Fahrzeug Status",
        "card_lane_id":"Spurwechsel-ID", "card_wait":"Warte...", "card_na":"n/a",
        "card_driving":"FÄHRT", "card_seam":"NAHT", "card_deloc":"⚠ DELOKALISIERT",
        "card_collision":"💥 KOLLISION", "card_recovery":"🔄 RECOVERY",
        "model_label":"Modell", "btn_apply_model":"MODELL ÜBERNEHMEN",
        "sw_firmware":"Firmware Override", "btn_clear":"Leeren",
        "auto_scroll":"Auto-Scroll", "car_connected":"Fahrzeug: Verbunden",
        "car_connecting":"Fahrzeug: Verbinde...", "car_disconnected":"Fahrzeug: Getrennt",
        "vehicle_prefix":"Aktives Fahrzeug", "auto_vehicle_set":"Fahrzeug auto-gesetzt:",
        "lang_btn":"🇬🇧 English",
        # Sidebar
        "sidebar_subtitle":"BLE Debug Dashboard",
        # Speed graph tab
        "sg_window_label":"Zeitfenster:",
        "sg_1min":"1 Min", "sg_2min":"2 Min", "sg_5min":"5 Min", "sg_full":"Alles",
        "sg_des_lbl":"Soll (mm/s)", "sg_act_lbl":"Ist (mm/s)",
        # Battery tab
        "batt_charge_indicator":"Ladestand-Anzeige (3,3 V – 4,2 V)",
        "batt_voltage_graph":"Spannung über Zeit (Ladevorgänge = goldene Markierung)",
        "batt_voltage_card":"Spannung", "batt_status_card":"Status",
        "batt_session_card":"Sitzungszeit",
        "batt_charging":"Lädt 🔌", "batt_discharging":"Entlädt", "batt_idle":"Bereit",
        "batt_unknown":"Unbekannt", "batt_volt_lbl":"Spannung (mV)",
        # Lap tab
        "lap_piece_label":"Start/Ziel-Stück-ID:", "placeholder_auto":"auto",
        "lap_btn_set":"Setzen", "lap_btn_reset":"Runden zurücksetzen",
        "lap_current_label":"Aktuell:", "lap_autodetect":"Start/Ziel: [automatisch]",
        "lap_autodetected":"Start/Ziel: Stück {} (auto)",
        "lap_piece_set":"Start/Ziel: Stück {}",
        "lap_best_times":"🏆 Bestzeiten",
        "lap_saved_hint":"💾 Gespeichert als times_*.csv",
        "lap_reset_timer":"⏱ --,-s",
        "lap_table_title":"Letzte 15 Runden", "lap_prefix":"Runde ",
        "lap_col_num":"#", "lap_col_time":"Zeit", "lap_col_gap":"Abstand", "lap_col_start":"Start",
        # Velocity tab
        "vel_speed_title":"Geschwindigkeit über Zeit",
        "vel_offset_title":"Spur-Versatz (Links ← 0 → Rechts)",
        "vel_act_lbl":"Ist-Geschw. (mm/s)", "vel_off_lbl":"Versatz (mm)", "vel_tgt_lbl":"Soll-Versatz (mm)",
        # Vehicle tab
        "veh_speed_label":"Geschw. mm/s:", "veh_accel_label":"Beschl. mm/s²:",
        "veh_send_speed":"Geschw. senden →", "veh_quick_cmds":"Schnellbefehle",
        "veh_ping":"Ping →", "veh_battery_req":"Batterie? →",
        "veh_sdk_mode":"SDK-Modus →", "veh_version_req":"Version? →",
        # About tab
        "about_version":"v2.9  ·  BLE Man-in-the-Middle Debug-Tool",
        "about_desc":"Ein inoffizielles Man-in-the-Middle BLE Debug-Tool für Anki Overdrive,\ngebaut mit zwei ESP32-Modulen und einem Python-HUD.",
        "about_feat1":"🔵  Fängt BLE-Traffic zwischen der Overdrive-App und Fahrzeugen ab",
        "about_feat2":"📡  Dekodiert und zeigt Pakete in Echtzeit an (Sniffer)",
        "about_feat3":"⚙️   Erlaubt Fahrzeug-ID- und Firmware-Änderungen zu Forschungszwecken",
        "about_feat4":"📊  Live-Graphen für Geschwindigkeit, Batterie und Versatz",
        "about_feat5":"🏁  Rundentimer mit automatischer Start/Ziel-Erkennung",
        "about_thanks":"Danke an MasterAirscrach — seine Protocol.cs ist die beste 🙏",
        "about_disclaimer_title":"⚠  Haftungsausschluss",
        "about_disclaimer":"OverCar Prime ist ein unabhängiges Fan-Projekt und steht in keiner Verbindung\nzu Anki, Inc. oder Digital Dream Labs.\nAlle Markenzeichen bleiben Eigentum ihrer jeweiligen Inhaber.\nDieses Tool wird ohne Gewähr für Bildungs- und Forschungszwecke bereitgestellt.\nBenutzung auf eigene Gefahr.",
        "about_tech_title":"Technologie",
        "about_tech1":"Python 3  ·  customtkinter  ·  pyserial",
        "about_tech2":"ESP32 (×2)  ·  Arduino BLE-Bibliotheken",
        "about_tech3":"Protokoll: MasterAirscrach / OverdriveServer",
        # Status / Details
        "status_recovered":"✅ Wiederhergestellt",
        "status_track":" (Strecke)", "status_low_batt":"Akku schwach! 🔴", "status_charged":"Voll geladen ✅",
        # Sniffer
        "sniff_time":"[Zeit]", "sniff_dir":"Richt.", "sniff_msg":"Nachricht", "sniff_info":"Info", "sniff_bytes":"Bytes"
    },
}

# =============================================================================
# MINI CANVAS GRAPH WIDGET
# =============================================================================

class LineGraph(tk.Canvas):
    """Lightweight tk.Canvas-based scrolling line graph. Supports multiple series."""

    def __init__(self, parent, bg="#1a1a2e", height=200, **kwargs):
        super().__init__(parent, bg=bg, height=height, highlightthickness=0, **kwargs)
        self.bg_col     = bg
        self.series     = {}   # name -> {"data": [(t, v)], "color": str, "label": str}
        self.window_sec = 60   # display window in seconds
        self.y_min      = None
        self.y_max      = None
        self.markers    = []   # [(t, label, color)]
        self._after_id  = None
        self.bind("<Configure>", self._on_resize)
        self._schedule_redraw()

    def add_series(self, name, color="#00BFFF", label=""):
        self.series[name] = {"data": [], "color": color, "label": label or name}

    def update_label(self, series_name, new_label):
        if series_name in self.series:
            self.series[series_name]["label"] = new_label

    def push(self, series_name, value, t=None):
        if series_name not in self.series:
            return
        if t is None:
            t = time.time()
        self.series[series_name]["data"].append((t, value))
        # Prune old data beyond 2× window to save memory
        cutoff = t - self.window_sec * 2
        self.series[series_name]["data"] = [
            p for p in self.series[series_name]["data"] if p[0] >= cutoff
        ]

    def add_marker(self, label, color="#FFD700", t=None):
        if t is None:
            t = time.time()
        self.markers.append((t, label, color))

    def set_window(self, seconds):
        self.window_sec = seconds

    def _on_resize(self, event):
        self._redraw()

    def _schedule_redraw(self):
        self._redraw()
        self._after_id = self.after(150, self._schedule_redraw)

    def _redraw(self):
        try:
            w = self.winfo_width()
            h = self.winfo_height()
        except:
            return
        if w < 10 or h < 10:
            return

        self.delete("all")
        PAD_L, PAD_R, PAD_T, PAD_B = 56, 14, 12, 28
        gw = w - PAD_L - PAD_R
        gh = h - PAD_T - PAD_B

        if gw < 10 or gh < 10:
            return

        now = time.time()
        t_end   = now
        t_start = now - self.window_sec

        # Collect all visible values for auto-scaling
        all_vals = []
        for s in self.series.values():
            for t_, v in s["data"]:
                if t_start <= t_ <= t_end:
                    all_vals.append(v)

        if self.y_min is not None and self.y_max is not None:
            y_lo, y_hi = self.y_min, self.y_max
        elif all_vals:
            y_lo = min(all_vals)
            y_hi = max(all_vals)
            pad  = max((y_hi - y_lo) * 0.1, 50)
            y_lo -= pad; y_hi += pad
        else:
            y_lo, y_hi = 0, 1000

        if y_hi == y_lo:
            y_hi = y_lo + 1

        def tx(t_): return PAD_L + (t_ - t_start) / self.window_sec * gw
        def ty(v):  return PAD_T + gh - (v - y_lo) / (y_hi - y_lo) * gh

        # Grid background
        self.create_rectangle(PAD_L, PAD_T, PAD_L+gw, PAD_T+gh, fill="#0d0d1a", outline="#2a2a4a")

        # Horizontal grid lines
        n_grid = 5
        for i in range(n_grid + 1):
            v   = y_lo + (y_hi - y_lo) * i / n_grid
            y_p = ty(v)
            self.create_line(PAD_L, y_p, PAD_L+gw, y_p, fill="#1e1e3a", dash=(3,4))
            self.create_text(PAD_L-4, y_p, text=f"{v:.0f}", anchor="e",
                             fill="#666688", font=("Consolas", 8))

        # Vertical time ticks
        n_t = min(6, self.window_sec // 10)
        for i in range(int(n_t) + 1):
            t_ = t_start + i * self.window_sec / n_t
            x  = tx(t_)
            self.create_line(x, PAD_T, x, PAD_T+gh, fill="#1e1e3a", dash=(3,4))
            rel = int(t_ - now)
            self.create_text(x, PAD_T+gh+3, text=f"{rel}s", anchor="n",
                             fill="#666688", font=("Consolas", 8))

        # Markers (lap lines, charge events)
        for mt, ml, mc in self.markers:
            if t_start <= mt <= t_end:
                x = tx(mt)
                self.create_line(x, PAD_T, x, PAD_T+gh, fill=mc, dash=(2,3), width=1)
                self.create_text(x+2, PAD_T+2, text=ml, anchor="nw",
                                 fill=mc, font=("Consolas", 7))

        # Zero line if in range
        if y_lo < 0 < y_hi:
            self.create_line(PAD_L, ty(0), PAD_L+gw, ty(0), fill="#444466", width=1)

        # Series lines
        legend_x = PAD_L + 4
        for sname, s in self.series.items():
            pts = [(tx(t_), ty(v)) for t_, v in s["data"] if t_start <= t_ <= t_end]
            if len(pts) >= 2:
                flat = [c for p in pts for c in p]
                self.create_line(*flat, fill=s["color"], width=2, smooth=False)
            if pts:
                lx, ly = pts[-1]
                self.create_oval(lx-3, ly-3, lx+3, ly+3, fill=s["color"], outline="")
            # Legend
            self.create_rectangle(legend_x, PAD_T+2, legend_x+10, PAD_T+10,
                                   fill=s["color"], outline="")
            self.create_text(legend_x+13, PAD_T+6, text=s["label"], anchor="w",
                              fill=s["color"], font=("Consolas", 9))
            legend_x += 100

        # Border
        self.create_rectangle(PAD_L, PAD_T, PAD_L+gw, PAD_T+gh, outline="#2a2a5a", width=1)


# =============================================================================
# MAIN HUD
# =============================================================================

class OverCarHUD(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.lang = "en"
        self.geometry("1400x950")
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

        # Telemetry state
        self._desired_speed  = 0
        self._actual_speed   = 0
        self._battery_mv     = 0
        self._offset_mm      = 0.0
        self._piece          = "N/A"
        self._lane_id        = 0
        self._on_charger     = False

        # Lap timer state
        self._lap_start_t    = None
        self._lap_times      = []          # list of (lap_num, duration_sec, start_ts)
        self._lap_count      = 0
        self._start_piece    = None        # piece ID detected as start/finish
        self._last_piece     = None
        self._car_connected_t= None

        # Lap file
        self._lap_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            now.strftime("times_%Y-%m-%d_%H-%M.csv")
        )

        self.tab_keys = ["tab_dash","tab_speed_graph","tab_battery","tab_laps",
                         "tab_velocity","tab_vehicle","tab_sniffer","tab_expert","tab_about"]
        self.current_tab_names = {}

        self.setup_ui()
        threading.Thread(target=self.background_discovery, daemon=True).start()
        self._start_clock()

    # ------------------------------------------------------------------
    # i18n
    # ------------------------------------------------------------------
    def t(self, k): return STRINGS[self.lang].get(k, k)

    def toggle_language(self):
        self.lang = "de" if self.lang == "en" else "en"
        self.apply_translations()

    def apply_translations(self):
        self.title(self.t("title"))
        self.btn_lang.configure(text=self.t("lang_btn"))
        self.lbl_sidebar_sub.configure(text=self.t("sidebar_subtitle"))
        self.btn_scan_cars.configure(text=self.t("btn_scan"))
        self.btn_connect_car.configure(text=self.t("btn_connect_car"))
        self.btn_clear_sniffer.configure(text=self.t("btn_clear"))
        self.sw_auto_scroll.configure(text=self.t("auto_scroll"))
        self.sw_firmware.configure(text=self.t("sw_firmware"))
        self.btn_apply_model.configure(text=self.t("btn_apply_model"))
        self.lbl_model_title.configure(text=self.t("model_label"))
        # Dashboard cards
        self.card_speed_des_lbl.configure(text=self.t("card_speed_des"))
        self.card_speed_act_lbl.configure(text=self.t("card_speed_act"))
        self.card_battery_lbl.configure(text=self.t("card_battery"))
        self.card_offset_lbl.configure(text=self.t("card_offset"))
        self.card_track_lbl.configure(text=self.t("card_track"))
        self.card_status_lbl.configure(text=self.t("card_status"))
        self.card_lane_id_lbl.configure(text=self.t("card_lane_id"))
        # ESP labels (only if not yet connected)
        if self.ser_app is None: self.lbl_esp_app.configure(text=self.t("esp_app_search"))
        if self.ser_car is None: self.lbl_esp_car.configure(text=self.t("esp_car_search"))
        # Speed graph tab
        self.lbl_sg_window.configure(text=self.t("sg_window_label"))
        for rb, key in self._sg_radio_btns:
            rb.configure(text=self.t(key))
        # Battery tab
        self.lbl_charge_indicator.configure(text=self.t("batt_charge_indicator"))
        self.lbl_voltage_graph.configure(text=self.t("batt_voltage_graph"))
        # Lap tab
        self.lbl_lap_table_title.configure(text=self.t("lap_table_title"))
        self.lbl_lap_piece.configure(text=self.t("lap_piece_label"))
        self.lap_piece_entry.configure(placeholder_text=self.t("placeholder_auto"))
        self.btn_lap_set.configure(text=self.t("lap_btn_set"))
        self.btn_lap_reset.configure(text=self.t("lap_btn_reset"))
        self.lbl_lap_current_label.configure(text=self.t("lap_current_label"))
        self.lbl_best_times.configure(text=self.t("lap_best_times"))
        self.lbl_lap_saved_hint.configure(text=self.t("lap_saved_hint"))
        # Update start piece label if still showing autodetect
        cur_sp = self.lbl_start_piece.cget("text")
        for l in ("en", "de"):
            if cur_sp == STRINGS[l]["lap_autodetect"]:
                self.lbl_start_piece.configure(text=self.t("lap_autodetect"))
                break
        # Velocity tab
        self.lbl_vel_speed_title.configure(text=self.t("vel_speed_title"))
        self.lbl_vel_offset_title.configure(text=self.t("vel_offset_title"))
        # Vehicle tab quick commands
        self.lbl_veh_quick_cmds.configure(text=self.t("veh_quick_cmds"))
        self.lbl_veh_speed.configure(text=self.t("veh_speed_label"))
        self.lbl_veh_accel.configure(text=self.t("veh_accel_label"))
        self.btn_send_speed.configure(text=self.t("veh_send_speed"))
        self.btn_ping.configure(text=self.t("veh_ping"))
        self.btn_batt_req.configure(text=self.t("veh_battery_req"))
        self.btn_sdk.configure(text=self.t("veh_sdk_mode"))
        self.btn_ver_req.configure(text=self.t("veh_version_req"))

        # Graphen Legenden aktualisieren
        self.speed_graph.update_label("desired", self.t("sg_des_lbl"))
        self.speed_graph.update_label("actual", self.t("sg_act_lbl"))
        self.batt_graph.update_label("voltage", self.t("batt_volt_lbl"))
        self.vel_speed_graph.update_label("actual", self.t("vel_act_lbl"))
        self.vel_offset_graph.update_label("offset", self.t("vel_off_lbl"))
        self.vel_offset_graph.update_label("target_offset", self.t("vel_tgt_lbl"))

        # Tab-Header updaten (CTk > v5.2 unterstützt tabview.rename)
        for tk_key in self.tab_keys:
            old_name = self.current_tab_names[tk_key]
            new_name = self.t(tk_key)
            if old_name != new_name:
                try:
                    self.tabview.rename(old_name, new_name)
                    self.current_tab_names[tk_key] = new_name
                except Exception:
                    pass

        # Dynamic card values that may still show static defaults
        cur_wait = self.card_status.cget("text")
        for l in ("en", "de"):
            if cur_wait == STRINGS[l]["card_wait"]:
                self.card_status.configure(text=self.t("card_wait"))
                break
            if cur_wait == STRINGS[l]["card_driving"]:
                self.card_status.configure(text=self.t("card_driving"))
                break
            if cur_wait == STRINGS[l]["card_seam"]:
                self.card_status.configure(text=self.t("card_seam"))
                break
            if cur_wait == STRINGS[l]["card_deloc"]:
                self.card_status.configure(text=self.t("card_deloc"))
                break
            if cur_wait == STRINGS[l]["card_collision"]:
                self.card_status.configure(text=self.t("card_collision"))
                break
            if cur_wait == STRINGS[l]["card_recovery"]:
                self.card_status.configure(text=self.t("card_recovery"))
                break
            if cur_wait == STRINGS[l]["status_recovered"]:
                self.card_status.configure(text=self.t("status_recovered"))
                break
            if cur_wait == STRINGS[l]["batt_unknown"]:
                self.batt_status_card.configure(text=self.t("batt_unknown"))
                break
            if self.card_track.cget("text") == STRINGS[l]["card_na"]:
                self.card_track.configure(text=self.t("card_na"))

        # App/car connection labels
        for old_key, new_key in [("lbl_car_disc","lbl_car_disc"),
                                   ("car_connected","car_connected"),
                                   ("car_disconnected","car_disconnected")]:
            for l in ("en","de"):
                if self.lbl_conn_car.cget("text") == STRINGS[l][old_key]:
                    self.lbl_conn_car.configure(text=self.t(new_key))
                    break
        for old_key in ("app_offline","app_online"):
            for l in ("en","de"):
                if self.lbl_app_status.cget("text") == STRINGS[l][old_key]:
                    self.lbl_app_status.configure(text=self.t(old_key))
                    break

        # Bestzeiten "Lap X" Labels neu zeichnen
        self._refresh_best_times()

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def init_log_file(self):
        try:
            with open(self.log_filename, "w", encoding="utf-8") as f:
                f.write(f"OVERCAR PRIME LOG: {datetime.datetime.now()}\n" + "="*80 + "\n")
        except: pass

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}\n"
        self.after(0, lambda: (self.txt_log.insert("end", line), self.txt_log.see("end")))

    def _start_clock(self):
        """Update live clock every second."""
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        try: self.lbl_clock.configure(text=now_str)
        except: pass
        self.after(1000, self._start_clock)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def setup_ui(self):
        self.title(self.t("title"))
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ---- SIDEBAR ----
        self.sidebar = ctk.CTkFrame(self, width=290, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_propagate(False)

        ctk.CTkLabel(self.sidebar, text="OVERCAR PRIME",
                     font=ctk.CTkFont(size=20, weight="bold")).pack(pady=(18,3))
        self.lbl_sidebar_sub = ctk.CTkLabel(self.sidebar, text=self.t("sidebar_subtitle"),
                     font=ctk.CTkFont(size=11), text_color="#888")
        self.lbl_sidebar_sub.pack(pady=(0,6))

        self.lbl_clock = ctk.CTkLabel(self.sidebar, text="--:--:--",
                                       font=ctk.CTkFont(family="Courier", size=18, weight="bold"),
                                       text_color="#00BFFF")
        self.lbl_clock.pack(pady=(0,8))

        self.btn_lang = ctk.CTkButton(self.sidebar, text=self.t("lang_btn"), width=120,
                                       fg_color="transparent", border_width=1,
                                       command=self.toggle_language)
        self.btn_lang.pack(pady=(0,6))

        self.lbl_esp_app = ctk.CTkLabel(self.sidebar, text=self.t("esp_app_search"), text_color="orange",
                                         font=ctk.CTkFont(size=11))
        self.lbl_esp_app.pack(fill="x", padx=18)
        self.lbl_esp_car = ctk.CTkLabel(self.sidebar, text=self.t("esp_car_search"), text_color="orange",
                                         font=ctk.CTkFont(size=11))
        self.lbl_esp_car.pack(fill="x", padx=18)

        ctk.CTkLabel(self.sidebar, text="─"*30, text_color="#333").pack(pady=5)

        self.manual_port_entry = ctk.CTkEntry(self.sidebar, placeholder_text=self.t("port_placeholder"), width=220)
        self.manual_port_entry.pack(pady=4, padx=18)

        bf = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        bf.pack(pady=4)
        ctk.CTkButton(bf, text="APP-ESP", width=88, command=lambda: self.force_connect(1)).grid(row=0, column=0, padx=4)
        ctk.CTkButton(bf, text="CAR-ESP", width=88, command=lambda: self.force_connect(2)).grid(row=0, column=1, padx=4)

        ctk.CTkLabel(self.sidebar, text="─"*30, text_color="#333").pack(pady=5)

        self.btn_scan_cars = ctk.CTkButton(self.sidebar, text=self.t("btn_scan"),
                                            command=self.start_ble_scan, state="disabled")
        self.btn_scan_cars.pack(pady=6, padx=18)

        self.car_dropdown = ctk.CTkComboBox(self.sidebar, values=[self.t("car_wait")], width=240)
        self.car_dropdown.pack(pady=4, padx=18)

        self.btn_connect_car = ctk.CTkButton(self.sidebar, text=self.t("btn_connect_car"),
                                              command=self.connect_to_selected_car, state="disabled")
        self.btn_connect_car.pack(pady=4, padx=18)

        self.lbl_conn_car = ctk.CTkLabel(self.sidebar, text=self.t("lbl_car_disc"), anchor="w",
                                          font=ctk.CTkFont(size=12))
        self.lbl_conn_car.pack(fill="x", padx=18, pady=(12,0))

        self.lbl_vehicle_id = ctk.CTkLabel(self.sidebar, text=self.t("lbl_vehicle_id"), anchor="w",
                                            font=ctk.CTkFont(size=11), text_color="#aaa")
        self.lbl_vehicle_id.pack(fill="x", padx=18, pady=(6,2))

        self.lbl_app_status = ctk.CTkLabel(self.sidebar, text=self.t("app_offline"),
                                            text_color="red", font=ctk.CTkFont(weight="bold"))
        self.lbl_app_status.pack(pady=4)

        # log path
        ctk.CTkLabel(self.sidebar, text=f"📄 {os.path.basename(self.log_filename)}",
                     font=ctk.CTkFont(size=9), text_color="#555").pack(fill="x", padx=12, pady=(8,0))

        # ---- MAIN AREA ----
        self.main_frame = ctk.CTkFrame(self, corner_radius=12)
        self.main_frame.grid(row=0, column=1, padx=16, pady=16, sticky="nsew")

        self.tabview = ctk.CTkTabview(self.main_frame)
        self.tabview.pack(fill="both", expand=True, padx=8, pady=8)

        for tk_key in self.tab_keys:
            name = self.t(tk_key)
            self.tabview.add(name)
            self.current_tab_names[tk_key] = name

        self._build_dashboard_tab()
        self._build_speed_graph_tab()
        self._build_battery_tab()
        self._build_lap_tab()
        self._build_velocity_tab()
        self._build_vehicle_tab()
        self._build_sniffer_tab()
        self._build_expert_tab()
        self._build_about_tab()

    # ------------------------------------------------------------------ DASHBOARD
    def _build_dashboard_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_dash"])
        tab.grid_columnconfigure((0,1,2), weight=1)
        tab.grid_rowconfigure((0,1,2), weight=1)

        self.card_speed_des, self.card_speed_des_lbl = self._dash_card(
            tab, self.t("card_speed_des"), "0 mm/s", 0, 0, "#00BFFF")
        self.card_speed_act, self.card_speed_act_lbl = self._dash_card(
            tab, self.t("card_speed_act"), "0 mm/s", 0, 1, "#00FF99")
        self.card_battery,   self.card_battery_lbl   = self._dash_card(
            tab, self.t("card_battery"),    "--",      0, 2, "#FFD700")
        self.card_offset,    self.card_offset_lbl    = self._dash_card(
            tab, self.t("card_offset"),    "0.0 mm",  1, 0, "#FF6EC7")
        self.card_track,     self.card_track_lbl     = self._dash_card(
            tab, self.t("card_track"),     self.t("card_na"),     1, 1, "#FFA500")
        self.card_lane_id,   self.card_lane_id_lbl   = self._dash_card(
            tab, self.t("card_lane_id"),   "0",       1, 2, "#BB88FF")
        self.card_status,    self.card_status_lbl    = self._dash_card(
            tab, self.t("card_status"), self.t("card_wait"), 2, 0, "#AAAAAA", colspan=3)

    def _dash_card(self, parent, title, initial, row, col, color, colspan=1):
        f = ctk.CTkFrame(parent, corner_radius=10)
        f.grid(row=row, column=col, columnspan=colspan, padx=8, pady=8, sticky="nsew")
        lbl_t = ctk.CTkLabel(f, text=title, text_color="gray", font=ctk.CTkFont(size=13))
        lbl_t.pack(pady=(14,3))
        lbl_v = ctk.CTkLabel(f, text=initial, font=ctk.CTkFont(size=26, weight="bold"), text_color=color)
        lbl_v.pack(pady=(0,14))
        return lbl_v, lbl_t

    # ------------------------------------------------------------------ SPEED GRAPH
    def _build_speed_graph_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_speed_graph"])
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)

        ctrl = ctk.CTkFrame(tab, fg_color="transparent")
        ctrl.grid(row=0, column=0, sticky="ew", padx=10, pady=8)

        self.lbl_sg_window = ctk.CTkLabel(ctrl, text=self.t("sg_window_label"), font=ctk.CTkFont(size=12))
        self.lbl_sg_window.pack(side="left", padx=(0,6))
        self._speed_window = ctk.IntVar(value=60)
        self._sg_radio_btns = []
        for label_key, sec in [("sg_1min",60),("sg_2min",120),("sg_5min",300),("sg_full",99999)]:
            rb = ctk.CTkRadioButton(ctrl, text=self.t(label_key), variable=self._speed_window, value=sec,
                               command=self._on_speed_window_change)
            rb.pack(side="left", padx=8)
            self._sg_radio_btns.append((rb, label_key))

        # Graph
        graph_frame = ctk.CTkFrame(tab, corner_radius=8)
        graph_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0,10))
        graph_frame.grid_columnconfigure(0, weight=1)
        graph_frame.grid_rowconfigure(0, weight=1)

        self.speed_graph = LineGraph(graph_frame, bg="#0d0d1a")
        self.speed_graph.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.speed_graph.add_series("desired", color="#00BFFF", label=self.t("sg_des_lbl"))
        self.speed_graph.add_series("actual",  color="#00FF99", label=self.t("sg_act_lbl"))
        self.speed_graph.y_min = 0

    def _on_speed_window_change(self):
        w = self._speed_window.get()
        self.speed_graph.set_window(w if w < 99999 else 99999)

    # ------------------------------------------------------------------ BATTERY TAB
    def _build_battery_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_battery"])
        tab.grid_columnconfigure((0,1,2), weight=1)
        tab.grid_rowconfigure(2, weight=1)

        # Status cards row
        self.batt_volt_card,     bl1 = self._dash_card(tab, self.t("batt_voltage_card"), "--",      0, 0, "#FFD700")
        self.batt_status_card,   bl2 = self._dash_card(tab, self.t("batt_status_card"),  self.t("batt_unknown"), 0, 1, "#AAAAAA")
        self.batt_session_card,  bl3 = self._dash_card(tab, self.t("batt_session_card"), "--",      0, 2, "#88BBFF")

        # Charge bar
        bar_f = ctk.CTkFrame(tab, corner_radius=8)
        bar_f.grid(row=1, column=0, columnspan=3, padx=10, pady=(0,8), sticky="ew")
        self.lbl_charge_indicator = ctk.CTkLabel(bar_f, text=self.t("batt_charge_indicator"),
                     font=ctk.CTkFont(size=11), text_color="#888")
        self.lbl_charge_indicator.pack(pady=(8,2))
        self.batt_progress = ctk.CTkProgressBar(bar_f, width=500, height=20,
                                                progress_color="#00FF99")
        self.batt_progress.set(0)
        self.batt_progress.pack(pady=(0,8), padx=20, fill="x")
        self.batt_pct_label = ctk.CTkLabel(bar_f, text="-- %", font=ctk.CTkFont(size=18, weight="bold"),
                                             text_color="#FFD700")
        self.batt_pct_label.pack(pady=(0,8))

        # Battery graph
        graph_f = ctk.CTkFrame(tab, corner_radius=8)
        graph_f.grid(row=2, column=0, columnspan=3, padx=10, pady=(0,10), sticky="nsew")
        graph_f.grid_columnconfigure(0, weight=1)
        graph_f.grid_rowconfigure(1, weight=1)
        self.lbl_voltage_graph = ctk.CTkLabel(graph_f, text=self.t("batt_voltage_graph"),
                     font=ctk.CTkFont(size=11), text_color="#888")
        self.lbl_voltage_graph.grid(row=0, column=0, pady=(6,2))
        self.batt_graph = LineGraph(graph_f, bg="#0d0d1a", height=180)
        self.batt_graph.grid(row=1, column=0, sticky="nsew", padx=4, pady=(0,4))
        self.batt_graph.add_series("voltage", color="#FFD700", label=self.t("batt_volt_lbl"))
        self.batt_graph.y_min = 3200
        self.batt_graph.y_max = 4300
        self.batt_graph.set_window(600)  # 10 min default for battery

        # Session start time tracking
        self._batt_session_start = None

    # ------------------------------------------------------------------ LAP TIMER
    def _build_lap_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_laps"])
        tab.grid_columnconfigure(0, weight=3)
        tab.grid_columnconfigure(1, weight=2)
        tab.grid_rowconfigure(1, weight=1)

        # Controls
        ctrl = ctk.CTkFrame(tab, fg_color="transparent")
        ctrl.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=8)

        self.lbl_lap_piece = ctk.CTkLabel(ctrl, text=self.t("lap_piece_label"), font=ctk.CTkFont(size=12))
        self.lbl_lap_piece.pack(side="left", padx=(0,6))
        self.lap_piece_entry = ctk.CTkEntry(ctrl, width=70, placeholder_text=self.t("placeholder_auto"))
        self.lap_piece_entry.pack(side="left", padx=4)
        self.btn_lap_set = ctk.CTkButton(ctrl, text=self.t("lap_btn_set"), width=60, command=self._lap_set_piece)
        self.btn_lap_set.pack(side="left", padx=4)
        self.btn_lap_reset = ctk.CTkButton(ctrl, text=self.t("lap_btn_reset"), width=140, fg_color="#8B0000",
                       command=self._lap_reset)
        self.btn_lap_reset.pack(side="left", padx=12)
        self.lbl_lap_current = ctk.CTkLabel(ctrl, text=self.t("lap_reset_timer"),
                                              font=ctk.CTkFont(family="Courier", size=22, weight="bold"),
                                              text_color="#00FF99")
        self.lbl_lap_current.pack(side="right", padx=12)
        self.lbl_lap_current_label = ctk.CTkLabel(ctrl, text=self.t("lap_current_label"), font=ctk.CTkFont(size=12), text_color="#888")
        self.lbl_lap_current_label.pack(side="right")

        self.lbl_start_piece = ctk.CTkLabel(ctrl, text=self.t("lap_autodetect"),
                                             font=ctk.CTkFont(size=11), text_color="#888")
        self.lbl_start_piece.pack(side="left", padx=16)

        # Left: recent laps table
        left = ctk.CTkFrame(tab, corner_radius=8)
        left.grid(row=1, column=0, padx=(10,4), pady=(0,10), sticky="nsew")
        left.grid_rowconfigure(1, weight=1)
        self.lbl_lap_table_title = ctk.CTkLabel(left, text=self.t("lap_table_title"),
                     font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_lap_table_title.grid(row=0, column=0, columnspan=4, pady=(10,6))
        # Header
        hf = ctk.CTkFrame(left, fg_color="#1a1a2e", corner_radius=4)
        hf.grid(row=1, column=0, sticky="ew", padx=8)
        left.grid_columnconfigure(0, weight=1)
        for i, (key, w) in enumerate([("lap_col_num",40),("lap_col_time",100),("lap_col_gap",90),("lap_col_start",140)]):
            ctk.CTkLabel(hf, text=self.t(key), width=w, font=ctk.CTkFont(size=11, weight="bold"),
                          text_color="#888").grid(row=0, column=i, padx=4, pady=2)
        self.lap_rows_frame = ctk.CTkScrollableFrame(left, corner_radius=4, height=400)
        self.lap_rows_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=(0,8))
        left.grid_rowconfigure(2, weight=1)
        self.lap_row_labels = []

        # Right: best times
        right = ctk.CTkFrame(tab, corner_radius=8)
        right.grid(row=1, column=1, padx=(4,10), pady=(0,10), sticky="nsew")
        self.lbl_best_times = ctk.CTkLabel(right, text=self.t("lap_best_times"),
                     font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_best_times.pack(pady=(12,6))

        self.best_frames = []
        medals = ["🥇", "🥈", "🥉"]
        for i in range(3):
            bf = ctk.CTkFrame(right, corner_radius=8, fg_color="#0d1a0d")
            bf.pack(fill="x", padx=12, pady=6)
            medal_lbl = ctk.CTkLabel(bf, text=medals[i], font=ctk.CTkFont(size=20))
            medal_lbl.pack(side="left", padx=10, pady=10)
            val_lbl = ctk.CTkLabel(bf, text="--", font=ctk.CTkFont(family="Courier", size=22, weight="bold"),
                                    text_color=["#FFD700","#C0C0C0","#CD7F32"][i])
            val_lbl.pack(side="left", padx=4)
            lap_lbl = ctk.CTkLabel(bf, text="", font=ctk.CTkFont(size=11), text_color="#666")
            lap_lbl.pack(side="left", padx=4)
            self.best_frames.append((val_lbl, lap_lbl))

        self.lbl_lap_saved_hint = ctk.CTkLabel(right, text=self.t("lap_saved_hint"),
                     font=ctk.CTkFont(size=10), text_color="#555")
        self.lbl_lap_saved_hint.pack(pady=(20,4))
        self.lbl_lap_file = ctk.CTkLabel(right, text=os.path.basename(self._lap_file),
                                          font=ctk.CTkFont(size=10), text_color="#444", wraplength=200)
        self.lbl_lap_file.pack(pady=(0,8))

        # Running lap timer
        self._update_lap_timer()

    def _update_lap_timer(self):
        if self._lap_start_t is not None:
            elapsed = time.time() - self._lap_start_t
            try: self.lbl_lap_current.configure(text=f"⏱ {elapsed:.2f}s")
            except: pass
        self.after(80, self._update_lap_timer)

    def _lap_set_piece(self):
        val = self.lap_piece_entry.get().strip()
        if val.isdigit():
            self._start_piece = int(val)
            self.lbl_start_piece.configure(text=self.t("lap_piece_set").format(self._start_piece))
            self.log(f"Lap: Start/finish piece set to {self._start_piece}")

    def _lap_reset(self):
        self._lap_times   = []
        self._lap_count   = 0
        self._lap_start_t = None
        self._refresh_lap_table()
        self._refresh_best_times()
        try: self.lbl_lap_current.configure(text=self.t("lap_reset_timer"))
        except: pass

    def _on_lap_trigger(self, piece_id):
        """Called when car crosses start/finish piece."""
        now = time.time()
        if self._start_piece is None:
            # Auto-set on first crossing
            self._start_piece = piece_id
            self.lbl_start_piece.configure(text=self.t("lap_autodetected").format(piece_id))
            self._lap_start_t = now
            self.log(f"Lap: Auto-detected start piece {piece_id}")
            return

        if piece_id != self._start_piece:
            return

        if self._lap_start_t is None:
            self._lap_start_t = now
            return

        lap_dur = now - self._lap_start_t
        if lap_dur < 1.0:
            return  # debounce: ignore < 1 second

        self._lap_count += 1
        start_ts = datetime.datetime.fromtimestamp(self._lap_start_t).strftime("%H:%M:%S.%f")[:-4]
        self._lap_times.append((self._lap_count, lap_dur, start_ts))
        self._lap_start_t = now
        self._save_lap_to_csv(self._lap_count, lap_dur, start_ts)
        self._refresh_lap_table()
        self._refresh_best_times()
        # Also mark on speed graph
        self.speed_graph.add_marker(f"L{self._lap_count}", color="#FFD700")
        self.log(f"Lap {self._lap_count}: {lap_dur:.3f}s")

    def _save_lap_to_csv(self, num, dur, start_ts):
        try:
            exists = os.path.isfile(self._lap_file)
            with open(self._lap_file, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                if not exists:
                    w.writerow(["Lap", "Time_s", "Start_timestamp"])
                w.writerow([num, f"{dur:.4f}", start_ts])
        except Exception as e:
            self.log(f"Lap save error: {e}")

    def _refresh_lap_table(self):
        for w in self.lap_rows_frame.winfo_children():
            w.destroy()
        recent = self._lap_times[-15:]
        best = min((d for _, d, _ in self._lap_times), default=None)
        for idx, (num, dur, start_ts) in enumerate(reversed(recent)):
            color = "#FFD700" if (best is not None and abs(dur - best) < 0.001) else (
                    "#cccccc" if idx == 0 else "#888888")
            row_bg = "#0d1a0d" if idx == 0 else "transparent"
            rf = ctk.CTkFrame(self.lap_rows_frame, fg_color=row_bg, corner_radius=4)
            rf.pack(fill="x", pady=1)
            gap_str = ""
            if best is not None and idx < len(recent)-1:
                gap = dur - best
                gap_str = f"+{gap:.3f}" if gap > 0 else f"{gap:.3f}"
            for i, (txt, w) in enumerate([
                (str(num), 40), (f"{dur:.3f}s", 100), (gap_str, 90), (start_ts, 140)
            ]):
                ctk.CTkLabel(rf, text=txt, width=w, font=ctk.CTkFont(
                    family="Courier", size=12, weight="bold" if idx==0 else "normal"),
                    text_color=color).grid(row=0, column=i, padx=4, pady=2)

    def _refresh_best_times(self):
        sorted_laps = sorted(self._lap_times, key=lambda x: x[1])
        for i, (vl, ll) in enumerate(self.best_frames):
            if i < len(sorted_laps):
                num, dur, _ = sorted_laps[i]
                vl.configure(text=f"{dur:.3f}s")
                ll.configure(text=f"{self.t('lap_prefix')}{num}")
            else:
                vl.configure(text="--")
                ll.configure(text="")

    # ------------------------------------------------------------------ VELOCITY/OFFSET
    def _build_velocity_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_velocity"])
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure((1,3), weight=1)

        self.lbl_vel_speed_title = ctk.CTkLabel(tab, text=self.t("vel_speed_title"),
                     font=ctk.CTkFont(size=13, weight="bold"), text_color="#888")
        self.lbl_vel_speed_title.grid(row=0, column=0, sticky="w", padx=14, pady=(10,2))

        vspeed_f = ctk.CTkFrame(tab, corner_radius=8)
        vspeed_f.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0,6))
        vspeed_f.grid_columnconfigure(0, weight=1)
        vspeed_f.grid_rowconfigure(0, weight=1)
        self.vel_speed_graph = LineGraph(vspeed_f, bg="#0d0d1a", height=200)
        self.vel_speed_graph.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.vel_speed_graph.add_series("actual", color="#00FF99", label=self.t("vel_act_lbl"))
        self.vel_speed_graph.set_window(60)
        self.vel_speed_graph.y_min = 0

        self.lbl_vel_offset_title = ctk.CTkLabel(tab, text=self.t("vel_offset_title"),
                     font=ctk.CTkFont(size=13, weight="bold"), text_color="#888")
        self.lbl_vel_offset_title.grid(row=2, column=0, sticky="w", padx=14, pady=(6,2))

        voffset_f = ctk.CTkFrame(tab, corner_radius=8)
        voffset_f.grid(row=3, column=0, sticky="nsew", padx=10, pady=(0,10))
        voffset_f.grid_columnconfigure(0, weight=1)
        voffset_f.grid_rowconfigure(0, weight=1)
        self.vel_offset_graph = LineGraph(voffset_f, bg="#0d0d1a", height=200)
        self.vel_offset_graph.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.vel_offset_graph.add_series("offset", color="#FF6EC7", label=self.t("vel_off_lbl"))
        self.vel_offset_graph.add_series("target_offset", color="#BB88FF", label=self.t("vel_tgt_lbl"))
        self.vel_offset_graph.set_window(60)
        self.vel_offset_graph.y_min = -100
        self.vel_offset_graph.y_max = 100

    # ------------------------------------------------------------------ VEHICLE TAB
    def _build_vehicle_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_vehicle"])
        self.lbl_model_title = ctk.CTkLabel(tab, text=self.t("model_label"), font=ctk.CTkFont(weight="bold"))
        self.lbl_model_title.pack(pady=10)
        self.model_dropdown = ctk.CTkComboBox(
            tab, values=[f"{hex(k).upper()}: {v}" for k,v in sorted(CAR_MODELS.items())], width=350)
        self.model_dropdown.pack(pady=5)
        self.model_dropdown.set(f"0x13: {CAR_MODELS[0x13]}")
        self.btn_apply_model = ctk.CTkButton(tab, text=self.t("btn_apply_model"), command=self.apply_id)
        self.btn_apply_model.pack(pady=5)
        self.sw_firmware = ctk.CTkSwitch(tab, text=self.t("sw_firmware"), command=self.send_version_settings)
        self.sw_firmware.pack(pady=15)
        self.fw_dropdown = ctk.CTkComboBox(
            tab, values=["5A.2E (Overdrive)","27.25 (Drive)","60.30 (MXT)"], width=300)
        self.fw_dropdown.pack(pady=5)
        self.fw_dropdown.set("5A.2E (Overdrive)")

        ctk.CTkLabel(tab, text="─"*50, text_color="gray").pack(pady=10)
        self.lbl_veh_quick_cmds = ctk.CTkLabel(tab, text=self.t("veh_quick_cmds"), font=ctk.CTkFont(weight="bold"))
        self.lbl_veh_quick_cmds.pack()

        sf = ctk.CTkFrame(tab, fg_color="transparent")
        sf.pack(pady=8)
        self.lbl_veh_speed = ctk.CTkLabel(sf, text=self.t("veh_speed_label"))
        self.lbl_veh_speed.grid(row=0, column=0, padx=5)
        self.entry_speed = ctk.CTkEntry(sf, width=100, placeholder_text="500")
        self.entry_speed.grid(row=0, column=1, padx=5)
        self.lbl_veh_accel = ctk.CTkLabel(sf, text=self.t("veh_accel_label"))
        self.lbl_veh_accel.grid(row=0, column=2, padx=5)
        self.entry_accel = ctk.CTkEntry(sf, width=100, placeholder_text="500")
        self.entry_accel.grid(row=0, column=3, padx=5)
        self.btn_send_speed = ctk.CTkButton(sf, text=self.t("veh_send_speed"), command=self.send_speed_cmd, width=140)
        self.btn_send_speed.grid(row=0, column=4, padx=10)

        pf = ctk.CTkFrame(tab, fg_color="transparent")
        pf.pack(pady=5)
        self.btn_ping     = ctk.CTkButton(pf, text=self.t("veh_ping"),        width=130, command=self.send_ping)
        self.btn_batt_req = ctk.CTkButton(pf, text=self.t("veh_battery_req"), width=130, command=self.send_battery_req)
        self.btn_sdk      = ctk.CTkButton(pf, text=self.t("veh_sdk_mode"),    width=130, command=self.send_sdk_mode)
        self.btn_ver_req  = ctk.CTkButton(pf, text=self.t("veh_version_req"), width=130, command=self.send_version_req)
        for i, btn in enumerate([self.btn_ping, self.btn_batt_req, self.btn_sdk, self.btn_ver_req]):
            btn.grid(row=0, column=i, padx=5)

    # ------------------------------------------------------------------ SNIFFER
    def _build_sniffer_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_sniffer"])
        ctrl = ctk.CTkFrame(tab, fg_color="transparent")
        ctrl.pack(fill="x", padx=10, pady=5)
        self.btn_clear_sniffer = ctk.CTkButton(ctrl, text=self.t("btn_clear"), width=100,
                                                command=lambda: self.txt_sniffer.delete("1.0","end"))
        self.btn_clear_sniffer.pack(side="left", padx=5)
        self.auto_scroll = ctk.BooleanVar(value=True)
        self.sw_auto_scroll = ctk.CTkSwitch(ctrl, text=self.t("auto_scroll"), variable=self.auto_scroll)
        self.sw_auto_scroll.pack(side="left", padx=20)
        self.txt_sniffer = ctk.CTkTextbox(tab, font=ctk.CTkFont(family="Consolas", size=11))
        self.txt_sniffer.pack(fill="both", expand=True, padx=10, pady=5)
        self.txt_sniffer.insert("end", f"{self.t('sniff_time'):14} {self.t('sniff_dir'):10} {self.t('sniff_msg'):25} {self.t('sniff_info'):<50} {self.t('sniff_bytes')}\n")
        self.txt_sniffer.insert("end", "─"*130 + "\n")

    # ------------------------------------------------------------------ EXPERT
    def _build_expert_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_expert"])
        self.txt_log = ctk.CTkTextbox(tab, font=ctk.CTkFont(family="Consolas", size=12))
        self.txt_log.pack(fill="both", expand=True, padx=10, pady=10)

    # ------------------------------------------------------------------ ABOUT
    def _build_about_tab(self):
        tab = self.tabview.tab(self.current_tab_names["tab_about"])
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(0, weight=1)

        outer = ctk.CTkScrollableFrame(tab, corner_radius=0, fg_color="transparent")
        outer.grid(row=0, column=0, sticky="nsew")
        outer.grid_columnconfigure(0, weight=1)

        # Hero card
        hero = ctk.CTkFrame(outer, corner_radius=12, fg_color="#0d0d1a")
        hero.pack(fill="x", padx=30, pady=(20,10))

        ctk.CTkLabel(hero, text="OverCar Prime",
                     font=ctk.CTkFont(family="Courier", size=32, weight="bold"),
                     text_color="#00BFFF").pack(pady=(20,4))
        ctk.CTkLabel(hero, text=self.t("about_version"),
                     font=ctk.CTkFont(size=13), text_color="#555").pack(pady=(0,8))

        ctk.CTkLabel(hero, text=self.t("about_desc"),
            font=ctk.CTkFont(size=13), text_color="#aaa", justify="center").pack(pady=(0,16))

        # Feature bullets
        feats = ctk.CTkFrame(hero, corner_radius=8, fg_color="#111122")
        feats.pack(fill="x", padx=24, pady=(0,16))
        for key in ["about_feat1","about_feat2","about_feat3","about_feat4","about_feat5"]:
            ctk.CTkLabel(feats, text=self.t(key), anchor="w",
                         font=ctk.CTkFont(size=12), text_color="#ccc").pack(
                fill="x", padx=20, pady=3)

        ctk.CTkLabel(hero, text=self.t("about_thanks"),
            font=ctk.CTkFont(size=13, weight="bold"), text_color="#FFD700").pack(pady=(0,20))

        # Disclaimer card
        disc = ctk.CTkFrame(outer, corner_radius=12, fg_color="#1a0d0d")
        disc.pack(fill="x", padx=30, pady=(0,10))
        ctk.CTkLabel(disc, text=self.t("about_disclaimer_title"),
                     font=ctk.CTkFont(size=14, weight="bold"), text_color="#FF6666").pack(pady=(16,6))
        ctk.CTkLabel(disc, text=self.t("about_disclaimer"),
            font=ctk.CTkFont(size=12), text_color="#cc9999", justify="center").pack(pady=(0,20))

        # Tech stack card
        tech = ctk.CTkFrame(outer, corner_radius=12, fg_color="#0d0d1a")
        tech.pack(fill="x", padx=30, pady=(0,20))
        ctk.CTkLabel(tech, text=self.t("about_tech_title"),
                     font=ctk.CTkFont(size=13, weight="bold"), text_color="#888").pack(pady=(14,6))
        for key in ["about_tech1", "about_tech2", "about_tech3"]:
            ctk.CTkLabel(tech, text=self.t(key), font=ctk.CTkFont(size=11), text_color="#666").pack(pady=2)
        ctk.CTkLabel(tech, text=" ", font=ctk.CTkFont(size=6)).pack()

    # ------------------------------------------------------------------
    # Sniffer output
    # ------------------------------------------------------------------
    def log_sniff(self, direction, payload):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        clean = "".join(c for c in payload if c in "0123456789ABCDEFabcdef")
        if not clean: return
        try:
            fb = [int(clean[i:i+2],16) for i in range(0, len(clean), 2)]
        except: return

        idx = 0
        while idx < len(fb):
            plen = fb[idx]; total = plen+1
            if idx+total > len(fb):
                frag = fb[idx:]
                self.after(0, lambda l=f"[{ts}] {direction:10} {'[Fragment]':25} {'[incomplete]':<50} {' '.join(f'{b:02X}' for b in frag)}":
                           self._sniffer_insert(l))
                break
            pkt  = fb[idx:idx+total]; idx += total
            mid  = pkt[1] if len(pkt)>=2 else -1
            name = MSG_SHORT.get(mid, f"0x{mid:02X}" if mid>=0 else "??")
            try:
                info, parsed = decode_packet_info(pkt)
            except Exception as e:
                info, parsed = f"[DecodeError: {e}]", None

            if parsed:
                self._update_dashboard(parsed)

            hex_str = " ".join(f"{b:02X}" for b in pkt)
            line    = f"[{ts}] {direction:10} {name:25} {info:<50} {hex_str}"
            self.after(0, lambda l=line: self._sniffer_insert(l))
            try:
                with open(self.log_filename, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except: pass

    def _sniffer_insert(self, line):
        self.txt_sniffer.insert("end", line + "\n")
        if self.auto_scroll.get():
            self.txt_sniffer.see("end")

    # ------------------------------------------------------------------
    # Dashboard update
    # ------------------------------------------------------------------
    def _update_dashboard(self, parsed):
        kind, d = parsed
        now = time.time()

        if kind == "speed":
            self._desired_speed = d["desiredSpeedMMPS"]
            self._actual_speed  = d["actualSpeedMMPS"]
            self.card_speed_des.configure(text=f"{d['desiredSpeedMMPS']} mm/s")
            self.card_speed_act.configure(text=f"{d['actualSpeedMMPS']} mm/s")
            # Graphs
            self.speed_graph.push("desired", d["desiredSpeedMMPS"])
            self.speed_graph.push("actual",  d["actualSpeedMMPS"])
            self.vel_speed_graph.push("actual", d["actualSpeedMMPS"])

        elif kind == "position":
            self._actual_speed = d["speedMMPS"]
            self._offset_mm    = d["offset"]
            piece              = d["segmentID"]
            self.card_speed_act.configure(text=f"{d['speedMMPS']} mm/s")
            self.card_offset.configure(text=f"{d['offset']:.1f} mm")
            self.card_track.configure(text=f"P:{d['segmentID']} / L:{d['locationID']}")
            self.card_status.configure(text=self.t("card_driving"), text_color="#00FF00")
            self.card_lane_id.configure(text=str(d["lastExecutedLaneChangeCmdID"]))
            # Graphs
            self.speed_graph.push("actual", d["speedMMPS"])
            self.vel_speed_graph.push("actual", d["speedMMPS"])
            self.vel_offset_graph.push("offset", d["offset"])
            # Lap detection
            if self._last_piece != piece:
                self._last_piece = piece
                if self._car_connected_t is not None:
                    self.after(0, lambda p=piece: self._on_lap_trigger(p))

        elif kind == "transition":
            self._offset_mm = d["offset"]
            self.card_offset.configure(text=f"{d['offset']:.1f} mm")
            self.card_track.configure(text=f"P:{d['currentSegmentIdx']} ← {d['previousSegmentIdx']}")
            self.card_status.configure(text=self.t("card_seam"), text_color="cyan")
            self.vel_offset_graph.push("offset", d["offset"])
            piece = d["currentSegmentIdx"]
            if self._last_piece != piece and self._car_connected_t is not None:
                self._last_piece = piece
                self.after(0, lambda p=piece: self._on_lap_trigger(p))

        elif kind == "offset":
            self._offset_mm = d["offsetFromCenterMM"]
            self.card_offset.configure(text=f"{d['offsetFromCenterMM']:.1f} mm")
            self.card_lane_id.configure(text=str(d["laneChangeID"]))
            self.vel_offset_graph.push("offset", d["offsetFromCenterMM"])

        elif kind == "lane_change":
            self.card_lane_id.configure(text=str(d["laneChangeID"]))
            self.card_offset.configure(text=f"{d['currentOffset']:.1f}→{d['targetOffset']:.1f} mm")
            self.vel_offset_graph.push("offset", d["currentOffset"])
            self.vel_offset_graph.push("target_offset", d["targetOffset"])

        elif kind == "battery":
            mv = d
            self._battery_mv = mv
            volts = mv / 1000.0
            bcolor = "#00FF99" if mv >= 3800 else ("#FFD700" if mv >= 3600 else "#FF4444")
            self.card_battery.configure(text=f"{volts:.2f} V\n({mv} mV)", text_color=bcolor)
            # Battery tab
            self.batt_volt_card.configure(text=f"{volts:.3f} V", text_color=bcolor)
            self.batt_graph.push("voltage", mv)
            # Progress bar: 3300 mV = 0%, 4200 mV = 100%
            pct = max(0.0, min(1.0, (mv - 3300) / (4200 - 3300)))
            self.batt_progress.set(pct)
            self.batt_progress.configure(progress_color=bcolor)
            self.batt_pct_label.configure(text=f"{pct*100:.0f} %", text_color=bcolor)
            if self._batt_session_start is None:
                self._batt_session_start = now
            elapsed = now - self._batt_session_start
            m, s = divmod(int(elapsed), 60)
            h, m = divmod(m, 60)
            self.batt_session_card.configure(text=f"{h:02d}:{m:02d}:{s:02d}")

        elif kind == "status":
            flags = []
            if d["onTrack"]:           flags.append(self.t("batt_charging").replace(" 🔌","") + self.t("status_track"))
            if d["onCharger"]:         flags.append(self.t("batt_charging"))
            if d["hasLowBattery"]:     flags.append(self.t("status_low_batt"))
            if d["hasChargedBattery"]: flags.append(self.t("status_charged"))
            text  = " | ".join(flags) if flags else self.t("batt_idle")
            color = "#FF4444" if d["hasLowBattery"] else ("#00FF99" if d["onTrack"] else "#AAAAAA")
            self.card_status.configure(text=text, text_color=color)
            self.batt_status_card.configure(text=self.t("batt_charging") if d["onCharger"] else
                                             (self.t("batt_discharging") if d["onTrack"] else self.t("batt_idle")))
            if d["onCharger"] and not self._on_charger:
                self.batt_graph.add_marker(self.t("batt_charging").replace(" 🔌",""), color="#FFD700")
            self._on_charger = d["onCharger"]

        elif kind == "delocalized":
            self.card_status.configure(text=self.t("card_deloc"), text_color="#FF4444")
        elif kind == "recovery_start":
            self.card_status.configure(text=self.t("card_recovery"), text_color="#FFA500")
        elif kind == "recovery_ok":
            self.card_status.configure(text=self.t("status_recovered"), text_color="#00FF99")
        elif kind == "collision":
            self.card_status.configure(text=self.t("card_collision"), text_color="#FF4444")

    # ------------------------------------------------------------------
    # Serial / BLE
    # ------------------------------------------------------------------
    def background_discovery(self):
        while self.is_discovering:
            for p in serial.tools.list_ports.comports():
                port = p.device
                if (self.ser_app and self.ser_app.port==port) or \
                   (self.ser_car and self.ser_car.port==port):
                    continue
                try:
                    s = serial.Serial(port, 500000, timeout=0.1)
                    time.sleep(2)
                    s.write(b"IDENT?\n")
                    end = time.time()+4
                    while time.time()<end:
                        if s.in_waiting:
                            line = s.readline().decode(errors="ignore").strip()
                            if "IDENT:OVERCAR_PRIME_APP" in line:
                                s.write(b"ACK_PRIME\n"); self.ser_app=s
                                self.after(0, lambda pt=port: self.lbl_esp_app.configure(
                                    text=f"🔌 APP-ESP: {pt}", text_color="green"))
                                threading.Thread(target=self.serial_thread, args=(s,"APP"), daemon=True).start()
                                break
                            elif "IDENT:OVERCAR_PRIME_CAR" in line:
                                s.write(b"ACK_PRIME\n"); self.ser_car=s
                                self.after(0, lambda pt=port: self.lbl_esp_car.configure(
                                    text=f"🔌 CAR-ESP: {pt}", text_color="green"))
                                self.after(0, lambda: (
                                    self.btn_scan_cars.configure(state="normal"),
                                    self.btn_connect_car.configure(state="normal")))
                                threading.Thread(target=self.serial_thread, args=(s,"CAR"), daemon=True).start()
                                break
                    if not (self.ser_app and self.ser_app.port==port) and \
                       not (self.ser_car and self.ser_car.port==port):
                        s.close()
                except: pass
            if self.ser_app and self.ser_car:
                self.is_discovering=False
            time.sleep(2)

    def serial_thread(self, ser, role):
        buf = bytearray()
        while ser.is_open:
            try:
                if ser.in_waiting:
                    buf.extend(ser.read(ser.in_waiting))
                    while b"\n" in buf:
                        idx = buf.index(b"\n")
                        line = buf[:idx].decode(errors="ignore").strip()
                        buf  = buf[idx+1:]
                        markers = ["RX_CAR=","RX_APP=","DEV=","MSG="]
                        found = []
                        for m in markers:
                            for match in re.finditer(re.escape(m), line):
                                found.append((match.start(), m))
                        found.sort()
                        if not found: continue
                        for i,(sp,cm) in enumerate(found):
                            ep  = found[i+1][0] if i+1<len(found) else len(line)
                            content = line[sp+len(cm):ep].strip()
                            if not content: continue
                            if cm=="RX_CAR=":
                                if self.ser_app: self.ser_app.write(f"TX_APP={content}\n".encode())
                                self.log_sniff("CAR→APP", content)
                            elif cm=="RX_APP=":
                                if self.ser_car: self.ser_car.write(f"TX_CAR={content}\n".encode())
                                self.log_sniff("APP→CAR", content)
                            elif cm=="DEV=":
                                p2 = content.split("|")
                                if len(p2)>=2:
                                    mac=p2[0]; mid=int(p2[1],16)
                                    fw=f"{p2[2]}.{p2[3]}" if len(p2)>3 else "?.?"
                                    self.discovered_cars[mac]={"name":CAR_MODELS.get(mid,f"0x{mid:02X}"),
                                                        "fw":fw,"model_id":mid}
                                    v=[f"{info['name']} | {m}" for m,info in self.discovered_cars.items()]
                                    self.after(0, lambda val=v: self.car_dropdown.configure(values=val))
                                    if mid==0x12: self.after(0, lambda m2=mid: self.auto_set_vehicle(m2))
                            elif cm=="MSG=":
                                self.log(f"[{role}] {content}")
                                if "Verbindung zum Auto OK" in content:
                                    self._car_connected_t = time.time()
                                    self.after(0, lambda: self.lbl_conn_car.configure(
                                        text=self.t("car_connected"), text_color="cyan"))
                                elif "Fehler" in content or "verloren" in content:
                                    self.after(0, lambda: self.lbl_conn_car.configure(
                                        text=self.t("car_disconnected"), text_color="white"))

                if role=="APP" and len(buf)>=6 and buf[0]==0xAA:
                    if buf[1]==0x01:
                        mid,state=buf[2],buf[3]
                        self.after(0, lambda m=mid, s=state: self.update_telemetry(m,s))
                        buf=buf[6:]
            except Exception as e:
                self.log(f"Serial Error [{role}]: {e}"); break
            time.sleep(0.005)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def auto_set_vehicle(self, model_id):
        name   = CAR_MODELS.get(model_id, f"0x{model_id:02X}")
        target = f"{hex(model_id).upper()}: {name}"
        all_v  = [f"{hex(k).upper()}: {v}" for k,v in sorted(CAR_MODELS.items())]
        if target in all_v: self.model_dropdown.set(target)
        if self.ser_app: self.ser_app.write(f"ID={model_id:02X}\n".encode())
        self.lbl_vehicle_id.configure(text=f"{self.t('vehicle_prefix')}: {name}")
        self.log(f"{self.t('auto_vehicle_set')} {name} (0x{model_id:02X})")

    def force_connect(self, esp_nr):
        port = self.manual_port_entry.get().upper()
        if not port: return
        try:
            s = serial.Serial(port, 921600, timeout=1.0)
            time.sleep(1.5); s.write(b"ACK_PRIME\n")
            if esp_nr==1:
                self.ser_app=s
                self.lbl_esp_app.configure(text=f"🔌 APP-ESP: {port}", text_color="green")
                threading.Thread(target=self.serial_thread, args=(s,"APP"), daemon=True).start()
            else:
                self.ser_car=s
                self.lbl_esp_car.configure(text=f"🔌 CAR-ESP: {port}", text_color="green")
                self.btn_scan_cars.configure(state="normal")
                self.btn_connect_car.configure(state="normal")
                threading.Thread(target=self.serial_thread, args=(s,"CAR"), daemon=True).start()
        except Exception as e: self.log(f"Force-connect failed: {e}")

    def start_ble_scan(self):
        if self.ser_car: self.ser_car.write(b"SCAN\n")

    def connect_to_selected_car(self):
        val = self.car_dropdown.get()
        if "|" in val:
            mac = val.split("|")[1].strip()
            self.ser_car.write(f"CONN={mac}\n".encode())
            name = self.discovered_cars.get(mac,{}).get("name","")
            self.after(0, lambda: self.lbl_conn_car.configure(
                text=f"{self.t('car_connecting').split(':')[0]}: {name}" if name else self.t("car_connecting"),
                text_color="orange"))

    def update_telemetry(self, mid, state):
        name = CAR_MODELS.get(mid, hex(mid))
        self.lbl_vehicle_id.configure(text=f"{self.t('vehicle_prefix')}: {name}")
        is_conn = bool(state & 0x80)
        self.lbl_app_status.configure(
            text=self.t("app_online") if is_conn else self.t("app_offline"),
            text_color="green" if is_conn else "red")

    def apply_id(self):
        val = self.model_dropdown.get()
        hid = val.split(":")[0].strip()[2:]
        if self.ser_app: self.ser_app.write(f"ID={hid}\n".encode())

    def send_version_settings(self):
        if not self.ser_app: return
        is_on = "1" if self.sw_firmware.get() else "0"
        fw    = self.fw_dropdown.get()
        maj,mn = ("5A","2E") if "5A.2E" in fw else ("27","25") if "27.25" in fw else ("60","30")
        self.ser_app.write(f"V_SPOOF={is_on},{maj},{mn},00\n".encode())

    def _send_to_car(self, pkt):
        if not self.ser_car: self.log("No CAR-ESP connected."); return
        hs = " ".join(f"{b:02X}" for b in pkt)
        self.ser_car.write(f"TX_CAR={hs}\n".encode())
        self.log_sniff("APP→CAR", hs)

    def send_ping(self):         self._send_to_car([0x01, MSG["SEND_PING"]])
    def send_battery_req(self):  self._send_to_car([0x01, MSG["SEND_BATTERY_VOLTAGE"]])
    def send_version_req(self):  self._send_to_car([0x01, MSG["SEND_REQUEST_CAR_VERSION"]])
    def send_sdk_mode(self):     self._send_to_car([0x03, MSG["SEND_SDK_MODE"], 0x01, 0x01])

    def send_speed_cmd(self):
        try:
            speed = int(self.entry_speed.get() or "500")
            accel = int(self.entry_accel.get() or "500")
        except ValueError: self.log("Invalid speed/accel value."); return
        self._send_to_car([0x06, MSG["SEND_SET_CAR_SPEED"],
                           speed&0xFF,(speed>>8)&0xFF, accel&0xFF,(accel>>8)&0xFF, 0x00])


if __name__ == "__main__":
    OverCarHUD().mainloop()
