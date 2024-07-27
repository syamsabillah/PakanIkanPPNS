#!/usr/bin/python3

# Import libraries
import pandas as pd
#from statsmodels.tsa.arima.model import ARIMA
import pymysql as innodb
import paho.mqtt.client as mqtt
import json
import datetime

innodb_connection = innodb.connect(host='sql6.freemysqlhosting.net', user='sql6703027', password='jVjpPldIsb', db='sql6703027')
cursor = innodb_connection.cursor()

# MQTT Settings 
MQTT_Broker = "broker.hivemq.com"
MQTT_Port = 1883
Keep_Alive_Interval = 60
Data_Topic = "pakanlele/data"
ReqData_Topic = "pakanlele/reqdata"
Control_Topic = "pakanlele/dbcontrol"
Circulation_Topic = "pakanlele/circulation"

# Subscribe
def on_connect(client, userdata, flags, rc):
  mqttc.subscribe(Data_Topic, 0)
  mqttc.subscribe(Control_Topic, 0)
  mqttc.subscribe(Circulation_Topic, 0)

def on_message(mosq, obj, msg):
  if msg.topic == Data_Topic:
    payload = msg.payload.decode()
    message = json.loads(payload)
    turbidity = message["Turbidity Kolam"]
    ph = message["pH Kolam"]
    dissolved_oxygen = message["Dissolved Oxygen"]
    print("Turbidity:", turbidity)
    print("pH:", ph)
    print("Dissolved Oxygen:", dissolved_oxygen)
    sql = "INSERT INTO sensor_data ( %s, %s, %s ) VALUES ( %f, %f, %f )" % ("turbidity", "ph", "dissolved_oxygen", float(turbidity), float(ph), float(dissolved_oxygen))

    # Save Data into DB Table
    try:
        cursor.execute(sql)
    except innodb.Error as error:
        print("Error: {}".format(error))
    innodb_connection.commit()

    # Query your data from MySQL
    query = "SELECT * FROM sensor_data"
    data = pd.read_sql(query, innodb_connection)

    # Fit ARIMA model
    if len(data) >= 10:
      # Replace p, d, q values with your chosen parameters
      p = 1  # lag value
      d = 1  # differencing order
      q = 1  # moving average window

      model_turbidity = ARIMA(data['turbidity'], order=(p, d, q))
      model_ph = ARIMA(data['ph'], order=(p, d, q))
      model_dissolved_oxygen = ARIMA(data['dissolved_oxygen'], order=(p, d, q))
      model_turbidity_fit = model_turbidity.fit()
      model_ph_fit = model_ph.fit()
      model_dissolved_oxygen_fit = model_dissolved_oxygen.fit()

      # Summary of the model
      print(model_turbidity_fit.summary())
      print(model_ph_fit.summary())
      print(model_dissolved_oxygen_fit.summary())

      # Forecast
      period_minute = 10
      period_day = 3
      forecast_len = (60 / period_minute) * (24 * period_day)  # Change this value to the number of periods you want to forecast
      forecast_turbidity = model_turbidity_fit.forecast(steps=int(forecast_len))
      forecast_ph = model_ph_fit.forecast(steps=int(forecast_len))
      forecast_dissolved_oxygen = model_dissolved_oxygen_fit.forecast(steps=int(forecast_len))

      # Assign forecast values to MySQL table
      try:
        cursor.execute("TRUNCATE TABLE forecast_data")
      except innodb.Error as error:
        print("Error: {}".format(error))
      innodb_connection.commit()
      for i in range(int(forecast_len)):
        sql = "INSERT INTO forecast_data ( %s, %s, %s ) VALUES ( %f, %f, %f )" % ("turbidity", "ph", "dissolved_oxygen", float(forecast_turbidity.values[i]), float(forecast_ph.values[i]), float(forecast_dissolved_oxygen.values[i]))
        try:
            cursor.execute(sql)
        except innodb.Error as error:
            print("Error: {}".format(error))
        innodb_connection.commit()

      # Assign circulation date to MySQL table
      for i in range(int(forecast_len)):
        time = data['timestamp'].values[len(data)-1] + (i+1)*10*60*1000000000
        timestring = pd.to_datetime(time, unit='ns').strftime('%Y-%m-%d %H:%M:%S')
        sql = "INSERT INTO circulation_data ( %s ) VALUES ( '%s' )" % ("next_circulation", timestring)
        try:
            cursor.execute("TRUNCATE TABLE circulation_data")
            cursor.execute(sql)
        except innodb.Error as error:
            print("Error: {}".format(error))
        innodb_connection.commit()
        if forecast_turbidity.values[i] > 15. or forecast_ph.values[i] < 5.5 or forecast_ph.values[i] > 7.5 or forecast_dissolved_oxygen.values[i] < 7.:
          time_send = {
            "Next Circulation": str(timestring)
          }
          time_json = json.dumps(time_send)
          mqttc.publish(Circulation_Topic, time_json)
          break
    else:
      print("Insufficient data points for forecasting. Minimum 10 data points required.")

  elif msg.topic == Control_Topic:
    payload = msg.payload.decode()
    message = json.loads(payload)
    print("Message:", message["message"])
    if message["message"] == "truncate_data":
      try:
        cursor.execute("TRUNCATE TABLE sensor_data")
      except innodb.Error as error:
        print("Error: {}".format(error))
      innodb_connection.commit()
    elif message["message"] == "truncate_forecast":
      try:
        cursor.execute("TRUNCATE TABLE forecast_data")
      except innodb.Error as error:
        print("Error: {}".format(error))
      innodb_connection.commit()
    elif message["message"] == "truncate_circulation":
      try:
        cursor.execute("TRUNCATE TABLE circulation_data")
      except innodb.Error as error:
        print("Error: {}".format(error))
      innodb_connection.commit()
    elif message["message"] == "request_data":
      sensor_data = pd.read_sql("SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 1", innodb_connection)
      if sensor_data.size > 0:
        send_data = {
          "Turbidity Kolam": sensor_data['turbidity'].values[-1],
          "pH Kolam": sensor_data['ph'].values[-1],
          "Dissolved Oxygen": sensor_data['dissolved_oxygen'].values[-1]
        }
      else:
        send_data = {
          "Turbidity Kolam": 0,
          "pH Kolam": 0,
          "Dissolved Oxygen": 0
        }
      send_data_json = json.dumps(send_data)
      mqttc.publish(ReqData_Topic, send_data_json)

      circulation = pd.read_sql("SELECT * FROM circulation_data ORDER BY id DESC LIMIT 1", innodb_connection)
      if circulation.size > 0:
        send_data = {
          "Next Circulation": str(circulation['next_circulation'].values[-1])
        }
      else:
        send_data = {
          "Next Circulation": str(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        }
      send_data_json = json.dumps(send_data)
      mqttc.publish(Circulation_Topic, send_data_json)
    else:
      return
    
  else:
    return

def on_subscribe(mosq, obj, mid, granted_qos):
  pass

mqttc = mqtt.Client()

# Assign event callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe

# Connect
#mqttc.tls_set(ca_certs="ca.crt", tls_version=ssl.PROTOCOL_TLSv1_2)
mqttc.connect(MQTT_Broker, int(MQTT_Port), int(Keep_Alive_Interval))

# Continue the network loop & close db-connection
mqttc.loop_forever()
innodb_connection.close()