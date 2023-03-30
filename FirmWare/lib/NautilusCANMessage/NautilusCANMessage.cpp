
#include <SimpleCanFacility.h>
#include <NautilusCANMessage.h>

int extract(int num, int hi, int lo)
{
    int range = (hi - lo + 1); //number of bits to be extracted
    //shifting a number by the number of bits it has produces inconsistent
    //results across machines so we need a special case for extract(num, 31, 0)
    if (range == 32)
        return num;
    uint32_t result = 0;
    //following the rule above, ((1 << x) - 1) << y) makes the mask:
    uint32_t mask = ((1 << range) - 1) << lo;
    //AND num and mask to get only the bits in our range
    result = num & mask;
    result = result >> lo; //gets rid of trailing 0s
    return result;
}

DecodedMessageID decodeMessageID(CanMessage msg)
{

    DecodedMessageID decoded;

    decoded.commandID = extract(msg.msgID, 28, 20);
    decoded.dataType = extract(msg.msgID, 19, 14);
    decoded.entity = extract(msg.msgID, 13, 10);
    decoded.multicast = extract(msg.msgID, 9, 9);
    decoded.nodeID = extract(msg.msgID, 8, 0);

    return decoded;
}

float NautilusCANMesage::extactFloat(CanMessage message){
    floatUnion extractUion;
    extractUion.bytesArray[3] = message.data[0];
    extractUion.bytesArray[2] = message.data[1];
    extractUion.bytesArray[1] = message.data[2];
    extractUion.bytesArray[0] = message.data[3];

    return extractUion.floatValue;
}

int NautilusCANMesage::extactInteger(CanMessage message){
    intUnion extractUion;
    extractUion.bytesArray[3] = message.data[0];
    extractUion.bytesArray[2] = message.data[1];
    extractUion.bytesArray[1] = message.data[2];
    extractUion.bytesArray[0] = message.data[3];

    return extractUion.intValue;
}

float NautilusCANMesage::getFloatPayload(){
    return extactFloat(msg);
}

int NautilusCANMesage::getIntPayload(){
    return extactInteger(msg);
}

bool NautilusCANMesage::getBoolPayLoad(){
    return extactBoolean(msg);

}
char NautilusCANMesage::getCharPayload(){
    return msg.data[4];

}    

bool NautilusCANMesage::extactBoolean(CanMessage msg){
    uint8_t value = msg.data[0];
    if (value == 0) {
        return false;
    }
    else {
        return true;
    }
}

int NautilusCANMesage::getNodeID(){
    return extract(msg.msgID, 8, 0);
}

int NautilusCANMesage::getMmulticast(){
    return extract(msg.msgID, 9, 9);
}

int NautilusCANMesage::getEntity(){
    return extract(msg.msgID, 13, 10);
}

int NautilusCANMesage::getDdataType(){
    return extract(msg.msgID, 19, 14);
}

int NautilusCANMesage::getCommandID(){
    return extract(msg.msgID, 28, 20);
}

NautilusCANMesage::NautilusCANMesage(){
    msg.isStandard = false;
}

void NautilusCANMesage::setRTR(bool isRTR){
    msg.isRTR = isRTR;
}

bool NautilusCANMesage::getRTR(){
    return msg.isRTR;
}
CanMessage NautilusCANMesage::getCanMessage(){
    return msg;
}

void NautilusCANMesage::setFloatPayload(float value){
    msg.dlc =4;
    floatUnion valueToSend;
    valueToSend.floatValue = value;
    
    msg.data[0] = valueToSend.bytesArray[0]; 
    msg.data[1] = valueToSend.bytesArray[1];  
    msg.data[2] = valueToSend.bytesArray[2]; 
    msg.data[3] = valueToSend.bytesArray[3];
}
void NautilusCANMesage::setIntPayload(uint32_t value){
    msg.dlc =4;
    intUnion valueToSend;
    valueToSend.intValue = value;
    
    msg.data[0] = valueToSend.bytesArray[0]; 
    msg.data[1] = valueToSend.bytesArray[1];  
    msg.data[2] = valueToSend.bytesArray[2]; 
    msg.data[3] = valueToSend.bytesArray[3];  
    
}

NautilusCANMesage::NautilusCANMesage(CanMessage message)
{
    msg = message;
    return;
}

CanMessage NautilusCANMesage::toCanMessage(){
    return msg;
}


void NautilusCANMesage::setEncodeMessageID(int commandID, int dataType, int entity, int multicast, int nodeID)
{
    msg.msgID = 0;
    msg.msgID = msg.msgID | ((commandID & 0x1FF) << 20);
    msg.msgID = msg.msgID | ((dataType & 0x3F) << 14);
    msg.msgID = msg.msgID | ((entity & 0xF) << 10);
    msg.msgID = msg.msgID | ((multicast & 0x1) << 9);
    msg.msgID = msg.msgID | (nodeID & 0x1FF);
}