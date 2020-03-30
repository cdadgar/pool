//address of running pool
const ADDR = 13;
const TEST = false;
//const TEST = true;

function initWebsocket(page){
  $('#message').show();
  $('#message').html('Connecting...');
  $('#controls').hide();
  
  if (TEST) {
	test();
    return;
  }

  var url;
  var host = location.hostname;
  if (location.protocol === 'https:') {
    url = 'wss://' + host + location.pathname.substring(0,location.pathname.indexOf('/',1)) + '_ws_/' + page;
  }
  else {
    if (host == 'localhost')
      host = '192.168.1.' + ADDR;
	
    var port = location.port;
    if (port === '')
      port = 81;
    else
      port = parseInt(port) + 1;

    url = 'ws://' + host + ':' + port + '/' + page;
  }
  
  console.log('ws url',url);
  
  socket = new WebSocket(url);
  socket.onopen = function(){
    console.log('onopen');
    $('#message').hide();
    $('#controls').show();
  }
  socket.onmessage = function(msg){
    message(msg);
  }
  socket.onclose = function(){
    console.log('onclose');
    $('#message').show();
    $('#message').html('Disconnected');
    $('#controls').hide();
  }
}

function test() {
  // simulate connection
  $('#message').hide();
  $('#controls').show();
  
  socket = {};
  socket.send = function(data) {
    console.log('send',data);
  }

  // simulate data coming on
  setTimeout(function(){
    var obj = {};
    
    // index page
//    obj.data = '{"command":"time", "value":"Sun 8:39am"}';
//    message(obj);	  
//    obj.data = '{"command":"pumpMode", "value":"2"}';
//    message(obj);	  
//    obj.data = '{"command":"solarMode", "value":"2"}';
//    message(obj);	  
//    obj.data = '{"command":"roofTemp", "value":"100"}';
//    message(obj);	  
//    obj.data = '{"command":"airTemp", "value":"85"}';
//    message(obj);	  
//    obj.data = '{"command":"waterTemp", "value":"91"}';
//    message(obj);	  
//    obj.data = '{"command":"targetTemp", "value":"93"}';
//    message(obj);
//    obj.data = '{"command":"pump", "value":"Off"}';
//    message(obj);	  
//    obj.data = '{"command":"solar", "value":"Off"}';
//    message(obj);	  
//    obj.data = '{"command":"pressure", "value":"40"}';
//    message(obj);
    
    // program page
//    obj.data = '{"pumpStart":"0", "pumpStop":"1", "targetTemp":"90", "pressureLow":"10", "pressureHigh":"50"}';
//    message(obj);	  

    // setup page
//    obj.data = '{"date":"5/29/16", "time":"11:01am", "timeout":"20", "sensor0":"0", "sensor1":"1", "sensor2":"2", "use_logging":"1", "logging_ip_addr":"10.1.10.237", "logging_ip_port":"80", "log_url":"some url"}';
//    message(obj);	  

    // client page
    obj.data = '{"command":"airTemp", "value":"85"}';
    message(obj);	  
  },1000);
}