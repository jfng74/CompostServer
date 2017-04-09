'''
Created on 2015-06-22

@author: caribou
'''


class CompostFanData:
    def __init__(self):
        self.node_temperature_surface = 0.0
        self.node_temperature_profondeur = 0.0
        self.node_1_battery_voltage = 0.0
        self.node_2_battery_voltage = 0.0
        self.node_3_battery_voltage = 0.0
        self.node_4_battery_voltage = 0.0
        self.node_1_temperature_ambiant = 0.0
        self.node_1_humidity_ambiant = 0.0
        self.node_1_temperature_surface = 0.0
        self.node_1_temperature_profondeur = 0.0
        self.node_2_temperature_surface = 0.0
        self.node_2_temperature_profondeur = 0.0
        self.node_3_temperature_surface = 0.0
        self.node_3_temperature_profondeur = 0.0
        self.node_4_temperature_surface = 0.0
        self.node_4_temperature_profondeur = 0.0
        self.moyenne_temperature_profondeur = 0.0
        self.moyenne_temperature_surface = 0.0


class NodeData:
    def __init__(self):
        self.node_id = 0
        self.timestamp = 0
        self.t_1 = 0.0
        self.t_2 = 0.0
        self.t_3 = 0.0
        self.t_4 = 0.0
        self.h_1 = 0.0
        self.bat_voltage = 0.0
        self.last_rssi = 0


class AllNode:
    def __init__(self):
        self.node_0 = NodeData()
        self.node_1 = NodeData()
        self.node_2 = NodeData()
        self.node_3 = NodeData()


class CompostFanConfig:
    def __init__(self):
        self.relais_mode = 0
        self.relais_etat = 0
        self.relais_delais = 0
        self.relais_t_moyenne = 0.0
        self.relais_consigne_temperature_fan = 0.0
        self.relais_consigne_offset_min_temperature_fan = 0.0
        self.relais_consigne_offset_max_temperature_fan = 0.0


class CompostRRDGRAPH:
    def __init__(self):
        self.graph_id = 0
        self.graph_start = ""
        self.graph_end = ""
