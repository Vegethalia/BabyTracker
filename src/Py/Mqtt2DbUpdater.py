import paho.mqtt.client as mqtt
import DbAccess as db
import Params
from datetime import datetime

TOPIC_DEBUG ="PChan/Debug"
TOPIC_BABYTRACKER = "babytracker/loc"

#dbuser=babytrack
#dbpass=@BabyTrack3r

def PublishText(topic, msg):
	theMsg="["+str(datetime.now())+ "]"+ msg
	_myClient.publish(topic, theMsg, 0, True)

def OnMessageReceived(client, userdata, msg):
	""" The callback for when a PUBLISH message is received from the server.
	The message will be inserted in the DB if connection is available. """
	msg.payload = msg.payload.decode("utf-8")
	theMsg="Received message ["+msg.topic+"]["+str(msg.payload)+"]"
	print(theMsg)
	PublishText(TOPIC_DEBUG, theMsg)

	#messages are like: "1, 1609515744, 41.472366, 2.043975, 0, 0.000000"
	trackMsg=str(msg.payload).split(",")
	if len(trackMsg) == 6:
		_MyDb.InsertNewEntry(trackMsg[0], trackMsg[1], trackMsg[2], trackMsg[3], trackMsg[4], trackMsg[5]) #we don't care if the insert fails....
		


##########################
# MAIN PROGRAM STARTS HERE 😊
########################## 
_myClient=mqtt.Client("PchanUpdater", False)
_myClient.on_message=OnMessageReceived
_myClient.connect("192.168.1.140", 1888) #"mitotoro.synology.me"

_myClient.subscribe(TOPIC_BABYTRACKER, )
PublishText(TOPIC_DEBUG, "Starting Python Updater")

_MyDb=db.BabyTrackerDB(Params.DB_USER, Params.DB_PASS, Params.DB_SERVER, Params.DB_DATABASE, Params.DB_PORT)

_myClient.loop_forever()
