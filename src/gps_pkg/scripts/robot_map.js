var map;
var polyLineCoords = [];
var lat;
var lng;
var image;
var marker;
var receivingLat = true;
var flightPath;
window.onload = function () {

  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Subscribing to a Topic

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/GPSChatter',
    messageType : 'std_msgs/Int32'
  });

  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
	if(receivingLat){
		lat = parseFloat(message.data)/10000000;
		receivingLat = false;
	}else{
		lng = parseFloat(message.data)/10000000;
		receivingLat = true;
		polyLineCoords.push({lat: lat , lng: lng});
			
		flightPath = new google.maps.Polyline({
    		path: polyLineCoords,
    		geodesic: true,
    		strokeColor: '#FF0000',
    		strokeOpacity: 1.0,
    		strokeWeight: 2
		});
		
        map.setCenter(polyLineCoords[polyLineCoords.length - 1]);

		marker.setMap(null);
		marker = new google.maps.Marker({
			position: polyLineCoords[polyLineCoords.length - 1],
			map: map,
			icon: image,
			title: 'Is This Australia?'
		});

		flightPath.setMap(map);
		
	}
  });

}


function initMap(){
    var myLatLng;
    if(polyLineCoords.length == 0){
        myLatLng = {lat: 35, lng: -120}; 
    }else{
        myLatLng = polyLineCoords[0]; 
    }
    

	map = new google.maps.Map(document.getElementById('map'), {
		center: myLatLng,
		zoom: 20
	});

	flightPath = new google.maps.Polyline({
    path: polyLineCoords,
    geodesic: true,
    strokeColor: '#FF0000',
    strokeOpacity: 1.0,
    strokeWeight: 2
	});
	
	flightPath.setMap(map)
	
	image = 'https://cdn1.iconfinder.com/data/icons/sports-volume-3/48/111-48.png'
	marker = new google.maps.Marker({
		position: myLatLng,
		map: map,
		icon: image,
		title: 'Is This Australia?'
	});


	//To Remove Marker, call marker.setMap(null)
}
