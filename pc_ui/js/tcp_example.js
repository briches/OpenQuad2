/*
In the node.js intro tutorial (http://nodejs.org/), they show a basic tcp 
server, but for some reason omit a client connecting to it.  I added an 
example at the bottom.
Save the following server in example.js:
*/

var net = require("net");

var server = net.createServer((socket) => {

  socket.on('error', (err) => {
    console.log(err);
  });

  socket.on('data', (dataBuffer) => {
    console.log('Received ', dataBuffer.length, ' bytes');
    console.log(dataBuffer.toString());
    dataBuffer.length
  });

  try {
    console.log("Connected peer: ", socket.remoteAddress);
    socket.pipe(socket);
  } catch (err) {}
});

server.listen(1337, "192.168.1.65");

