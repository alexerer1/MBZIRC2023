#include "aruco_map.h"

#include <cmath>
#include <libxml/parser.h>
#include <libxml/xmlmemory.h>

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Convert the xmlChar array read from
* the xml file to an integer
*
****************************************/
int str2int(const xmlChar *str){
	int value = 0;

	int len = xmlStrlen(str);
	int multiplier = 1;
	int offset = 0;

	// Account for negative numbers
	if(str[0] == '-'){
		multiplier = -1;
		offset = 1;
	}

	// Convert to integer
	for(int i = len-1; i >= offset; i--){
		value += multiplier*(str[i]-'0')*std::pow(10.0,len-1 - i);
	}

	return value;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Convert the xmlChar array read from
* the xml file to a float
*
****************************************/
float str2float(const xmlChar *str){
	
	float value = 0.0f;
	int multiplier = 1;
	int offset = 0;
	int len = xmlStrlen(str);
	int dec;
	
	// Account for negative numbers
	if(str[0] == '-'){
		multiplier = -1;
		offset = 1;
	}

	// Find the decimal point
	for(int i = 0; i < len; i++){
		if(str[i] == '.'){
			dec = i;
			break;
		}
		else{
			dec = -1;
		}
	}
	// If no decimal point treat as integer
	if(dec == -1){
		return (float)str2int(str);
	}
	// Calculate value from string
	else{
		for(int i = dec-1; i >= offset; i--){
			value += multiplier*(str[i]-'0')*std::pow(10.0,dec-1 - i);
		}
		for(int i = len-1; i > dec; i--){
			value += multiplier*(str[i]-'0')/std::pow(10.0,i-dec);
		}
	}
	return value;	
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


/****************************************
*
* Return the index in the array of the
* marker with a specific id.
* Returns -1 if id not found
*
****************************************/
int ar_map::get_index_from_id(int id){
	for(int i = 0; i < num_markers; i++){
		if( ids[i] == id )
			return i;
	}
	return -1;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Return number of markers
*
****************************************/
int ar_map::get_num_markers(void){ return num_markers;}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Return marker for a given marker id
*
****************************************/
ar_marker ar_map::get_marker_from_id(int id){
	int index = get_index_from_id(id);
	return markers[index];
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Return marker from index in the marker
* array
*
****************************************/
ar_marker ar_map::get_marker_from_index(int index){
	return markers[index];
}


//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Calc the radius around the marker
* visibe by the drone depending on
* FOV and height
*
****************************************/
float ar_map::calc_marker_visibility_radius(float fov, float height, int marker_id){
	fov *= M_PI/180.0;
	ar_marker marker = get_marker_from_id(marker_id);

	float visible_diag = height*tan(fov);

	return (visible_diag - marker.size/2);
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Return marker size of a given marker
* id
*
****************************************/
float ar_map::get_marker_size_from_id(int id){
	return get_marker_from_id(id).size;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/****************************************
*
* Decode the map describes in the file
* with the information from the xml file
*
****************************************/
int ar_map::decode_xml(const char * filename) {

	xmlDocPtr doc;
	xmlNodePtr cur;
	xmlChar * data;

	doc = xmlParseFile(filename);

	cur = xmlDocGetRootElement(doc);
	cur = cur->xmlChildrenNode;

	// Find the number of markers to be included in the map (takes the n first)
	while(cur != NULL){
		if(!xmlStrcmp(cur->name, (const xmlChar *)"num_markers")){
			data = xmlNodeListGetString(doc,cur->xmlChildrenNode,1);
			num_markers = str2int(data);
			break;
		}
		cur = cur->next;
	}
	printf("Decoding map with %d markers\n\n",num_markers);

	// Initialize a marker
	num_markers = num_markers;
	markers = new ar_marker[num_markers];
	ids = new int[num_markers];

	// Extract information about each marker and store in the map array
	for(int i = 0; i < num_markers; i++) {
		
		// Move to next marker element if present and determine type of marker
		while(cur != NULL){
			if(!xmlStrcmp(cur->name, (const xmlChar *)"marker")){
				data = xmlGetProp(cur,(const xmlChar *)"type");
				if(!xmlStrcmp(data, (const xmlChar *)"fixed")) markers[i].fixed=true;
				else markers[i].fixed=false;
				break;
			}
			cur = cur->next;
		}
		// break if no more markers found in map file
		if(cur == NULL){
			num_markers = i;
			break;
		}

		// Decode all parameters of the marker
		xmlNodePtr child = cur->xmlChildrenNode;
		child = child->next;
		data = xmlNodeListGetString(doc,child->xmlChildrenNode,1);
		markers[i].id = str2int(data);
		child = child->next;
		child = child->next;
		data = xmlNodeListGetString(doc,child->xmlChildrenNode,1);
		markers[i].x = str2float(data);
		child = child->next;
		child = child->next;
		data = xmlNodeListGetString(doc,child->xmlChildrenNode,1);
		markers[i].y = str2float(data);
		child = child->next;
		child = child->next;
		data = xmlNodeListGetString(doc,child->xmlChildrenNode,1);
		markers[i].z = str2float(data);
		child = child->next;
		child = child->next;
		data = xmlNodeListGetString(doc,child->xmlChildrenNode,1);	
		markers[i].th = str2float(data);
		child = child->next;
		child = child->next;
		data = xmlNodeListGetString(doc,child->xmlChildrenNode,1);	
		markers[i].size = str2float(data);

		// Update id array
		ids[i] = markers[i].id;

		cur = cur->next;
	}


	xmlFreeDoc(doc);
	xmlCleanupParser();
	xmlMemoryDump();
	return 1;
}
