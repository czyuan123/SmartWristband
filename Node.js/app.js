 'use strict';
//import module
const express = require('express');
const mqtt = require('mqtt');
const admin = require('firebase-admin');
const serviceAccount = require('./serviceAccountKey.json');

const app = express();

admin.initializeApp({
    credential : admin.credential.cert(serviceAccount)
});

//variable for firestore and mqtt
const db = admin.firestore();
db.settings({
  timestampsInSnapshots: true,
});

// create a server on host 3000 and Start the server
const PORT = process.env.PORT || 4000;
app.listen(PORT, () => {
    app.use(express.static('public'));
  console.log(`App listening on port ${PORT}`);
  console.log('Press Ctrl+C to quit.');
});


//Setup variables for the MQTT communication
var MQTT_TOPIC = "wristband/client";
var MQTT_ADDR = "mqtt://broker.xxx.:1883";
var MQTT_PORT = 1883;
var client = mqtt.connect(MQTT_ADDR);

// prints date & time in YYYY-MM-DD format

var date_ob = new Date();
let date = date_ob.getDate();
let month = date_ob.getMonth() + 1;
let year = date_ob.getFullYear();
// current hours
let hours = date_ob.getHours() + 8;
// current minutes
let minutes = date_ob.getMinutes();
// current seconds
let seconds = date_ob.getSeconds();

// prints date & time in YYYY-MM-DD HH:MM:SS format
var time = (year + "-" + month + "-" + date + " " + hours + ":" + minutes + ":" + seconds);
console.log(time);

// [END gae_node_request_example]
client.on('connect', function () {
    client.subscribe(MQTT_TOPIC, {qos:1} );
        console.log("connected");
  });
   
  client.on('message', function (topic, message) {
    // message is Buffer
    var jsonObj = JSON.parse(message);
    console.log(jsonObj , Date);
     const Data = {
    Latitude : jsonObj.Latitude,
    Longitude : jsonObj.Longitude,
    Temperature : jsonObj.Temperature,
    HeartRate : jsonObj.HeartRate,
    Battery : jsonObj.Battery,
    Status :jsonObj.Status,
    isWorn :jsonObj.isWorn,
    Time : time
    };
  
  return db.collection('Quarantine').doc('Patient').collection('John').add(Data)
  .then(()=>{console.log('Data added');
  });

  app.get('/data/mkrgsm1400', (req, res) => {
  res.status(200).send('Hello, world!').end();
  res.send(Data)
})
  })

  client.on('error',function(error){
    console.log("Can't connect" + error);
    process.exit(1)
    });

module.exports = app;
