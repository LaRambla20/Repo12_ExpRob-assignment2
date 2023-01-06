/**
 * \file marker_server.cpp
 * \author Carmine Recchiuto
 * \date December 2022
 * \version 0.1
 * \brief Server node used for the acquisition of information from aruco markers.
 * \details
 *
 * Services : <br>
 *  /room_info
 *
 * Description:
 * This node implements a server that, given the ID of a marker, returns information about the location associated with that marker (if any).
 * In particular, the provided information consists in: location label, location coordinates, labels of the adjacent locations and corresponding doors.
 */

#include <ros/ros.h>
#include <assignment2/RoomConnection.h>
#include <assignment2/RoomInformation.h>


/**
* \brief Function that is called every time that a request message belonging to the '/room_info' service is received.
* \param req variable containing the service request passed by reference
* \param res variable containing the service response passed by reference
* \return true
*
* This function analyses the marker ID received as a request and, based on its value, sends a response, containing information about the location associated with that marker (if any).
*/

bool markerCallback(assignment2::RoomInformation::Request &req, assignment2::RoomInformation::Response &res){
	assignment2::RoomConnection conn;
	switch (req.id){
	case 11:
		res.room = "E";
		res.x = 1.5;
		res.y = 8.0;
		conn.connected_to = "C1";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		break;
	case 12: 
		res.room = "C1";
		res.x = -1.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		conn.connected_to = "R2";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 13: 
		res.room = "C2";
		res.x = 3.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		conn.connected_to = "C1";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R3";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		conn.connected_to = "R4";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	case 14: 
		res.room = "R1";
		res.x = -7.0;
		res.y = 3.0;
		conn.connected_to = "C1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		break;
	case 15: 
		res.room = "R2";
		res.x = -7.0;
		res.y = -4.0;
		conn.connected_to = "C1";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 16: 
		res.room = "R3";
		res.x = 9.0;
		res.y = 3.0;
		conn.connected_to = "C2";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		break;
	case 17: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	case 155: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	default:
		res.room = "no room associated with this marker id";
	}
	return true;
}	



/**
* \brief Main function.
* \return 0
*
* This function simply initialises the node and the node handle, defines and initialises the server and spins to allow the cyclical execution of this mechanism.
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "assignment2");
	ros::NodeHandle nh;
	ros::ServiceServer oracle = nh.advertiseService( "/room_info",markerCallback); /**< server belonging to the /room_info topic */
	ros::spin();
	ros::shutdown();
	return 0;
}
