
'use strict';
const express = require('express');
const app = express();

app.get('/', (req, res) => {
    res.status(200).send('Hello, world!').end();
  });

const admin = require('firebase-admin');
const serviceAccount = require ('./serviceAccountKey.json');

admin.initializeApp({
    credential : admin.credential.cert(serviceAccount)
});

const db = admin.firestore();
var http = require('https');
var url = require('url');
var dataa={"quote":"",
	"author":""
	};


// Start the server
const PORT = process.env.PORT || 8080;
app.listen(PORT, () => {
  console.log(`App listening on port ${PORT}`);
  console.log('Press Ctrl+C to quit.');
});
// [END gae_node_request_example]

module.exports = app;