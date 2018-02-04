var map;
var polyLineCoords = [];
var image;
var marker;
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
    name : '/gps_data_coords',
    messageType : 'gps_pkg/GPS_Coord'
  });


   listener.subscribe(function(message) {
      console.log('Received message on ' + listener.name + ': ' + message.data);
      lat = parseFloat(message.lat)/10000000;
      lng = parseFloat(message.lng)/10000000;
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
