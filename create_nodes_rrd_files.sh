#/bin/bash

# Le repertoire qui contiendra les fichiers rrd
#rrd_dir='/mnt/RAMdata/rrd_files/'
rrd_dir='./'

/usr/bin/rrdtool create $rrd_dir/node_00.rrd  --step 300 \
DS:t_surface:GAUGE:400:-40:120 \
DS:t_profondeur:GAUGE:400:-40:120 \
DS:t_air:GAUGE:400:-40:120 \
DS:humidity:GAUGE:400:0:120 \
DS:batt_voltage:GAUGE:400:0:5 \
DS:last_rssi:GAUGE:400:-100:0 \
RRA:AVERAGE:0.5:1:1200 \
RRA:AVERAGE:0.5:12:2400 \
RRA:AVERAGE:0.1:36:8785 \
RRA:MIN:0.1:36:8785 \
RRA:MAX:0.1:36:8785

/usr/bin/rrdtool create $rrd_dir/node_01.rrd  --step 300 \
DS:t_surface:GAUGE:400:-40:120 \
DS:t_profondeur:GAUGE:400:-40:120 \
DS:batt_voltage:GAUGE:400:0:5 \
DS:last_rssi:GAUGE:400:-100:0 \
RRA:AVERAGE:0.5:1:1200 \
RRA:AVERAGE:0.5:12:2400 \
RRA:AVERAGE:0.1:36:8785 \
RRA:MIN:0.1:36:8785 \
RRA:MAX:0.1:36:8785

/usr/bin/rrdtool create $rrd_dir/node_02.rrd  --step 300 \
DS:t_surface:GAUGE:400:-40:120 \
DS:t_profondeur:GAUGE:400:-40:120 \
DS:batt_voltage:GAUGE:400:0:5 \
DS:last_rssi:GAUGE:400:-100:0 \
RRA:AVERAGE:0.5:1:1200 \
RRA:AVERAGE:0.5:12:2400 \
RRA:AVERAGE:0.1:36:8785 \
RRA:MIN:0.1:36:8785 \
RRA:MAX:0.1:36:8785

/usr/bin/rrdtool create $rrd_dir/node_03.rrd  --step 300 \
DS:t_surface:GAUGE:400:-40:120 \
DS:t_profondeur:GAUGE:400:-40:120 \
DS:batt_voltage:GAUGE:400:0:5 \
DS:last_rssi:GAUGE:400:-100:0 \
RRA:AVERAGE:0.5:1:1200 \
RRA:AVERAGE:0.5:12:2400 \
RRA:AVERAGE:0.1:36:8785 \
RRA:MIN:0.1:36:8785 \
RRA:MAX:0.1:36:8785

/usr/bin/rrdtool create $rrd_dir/node_FE.rrd  --step 300 \
DS:setpoint_relais:GAUGE:400:0:100 \
DS:etat_relais:GAUGE:400:0:1 \
DS:node_01_rssi:GAUGE:400:-100:0 \
DS:node_02_rssi:GAUGE:400:-100:0 \
DS:node_03_rssi:GAUGE:400:-100:0 \
DS:node_04_rssi:GAUGE:400:-100:0 \
DS:t_avg:GAUGE:400:-40:120 \
RRA:AVERAGE:0.5:1:1200 \
RRA:AVERAGE:0.5:12:2400 \
RRA:AVERAGE:0.1:36:8785 \
RRA:MIN:0.1:36:8785 \
RRA:MAX:0.1:36:8785
