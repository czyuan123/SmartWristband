const functions = require("firebase-functions");
const admin = require("firebase-admin");
const mqtt = require("mqtt");
const express = require("express");

const app = express();
admin.initializeApp();

app.on("listening", function() {
  console.log("Server is listening");
});
/* exports.helloWorld = functions.https.onRequest((request, response) => {
  functions.logger.info("Hello logs!", {structuredData: true});
  response.send("Hello from Firebase!");
});*/

// variable for firestore and mqtt
const db = admin.firestore();

db.settings({
  timestampsInSnapshots: true,
});

// Setup variables for the MQTT communication
const MQTT_TOPIC = "wristband/client";
const MQTT_ADDR = "mqtt://broker.emqx.io:1883";
// const MQTT_PORT = 1883;
const client = mqtt.connect(MQTT_ADDR);

// prints date & time in YYYY-MM-DD format
const dateob = new Date();
const date = dateob.getDate();
const month = dateob.getMonth() + 1;
const year = dateob.getFullYear();
// current hours
const hours = dateob.getHours() + 8;
// current minutes
const minutes = dateob.getMinutes();
// current seconds
const seconds = dateob.getSeconds();

// prints date & time in YYYY-MM-DD HH:MM:SS format
const time =
  year + "-" + month + "-" + date + " " + hours + ":" + minutes + ":" + seconds;
console.log(time);

// var window;
let userId;
let id;
exports.getid = functions.firestore
    .document("Users/{userId}")
    .onWrite((change, context) => {
      userId = context.params.userId;

      const collectionref = db.collection("Users");
      const docref = collectionref.doc(userId).collection("Data");

      docref.doc("Profile")
          .set({Quarantine: time}, {merge: true})
          .then(() => {
            console.log("successfully added");
            return null;
          })
          .catch((error) => {
            return null;
          });

      change.after.exists ? change.after.data() : null;

      // Get an object with the previous document value (for update or delete)
      const newDocument = change.after.data();
      id = newDocument.id;
      console.log("id = " + id);
      client.on("message", (topic, message) => {
        // message is Buffer
        const jsonObj = JSON.parse(message);
        console.log(jsonObj, Date);
        // eslint-disable-next-line no-unused-vars
        const Data = {
          Latitude: jsonObj.Latitude,
          Longitude: jsonObj.Longitude,
          Temperature: jsonObj.Temperature,
          HeartRate: jsonObj.HeartRate,
          Battery: jsonObj.Battery,
          isWorn: jsonObj.isWorn,
          Status: jsonObj.Status,
          Time: time,
        };
        return db
            .collection("Users")
            .doc(id)
            .collection("Quarantine")
            .add(Data)
            .then(() => {
              console.log("Data added");
            });
      });
    });


// [END gae_node_request_example]
client.on("connect", () => {
  client.subscribe(MQTT_TOPIC, {qos: 1});
  console.log("connected");
});

client.on("error", (error) => {
  console.log("Can't connect" + error);
  process.exit(1);
});

app.post("/", (req, res)=>{
  const phonenum = req.body.PhoneNumber;
  console.log(phonenum);
});
// app.listen(8080);

