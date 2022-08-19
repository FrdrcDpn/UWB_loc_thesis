#include <string.h>
#include "lpsTwrTag.h"
#include "log.h"
#include "debug.h"
#include "physicalConstants.h"
#include "FreeRTOS.h"
#include "task.h"
#include "configblock.h"
#include "estimator_kalman.h"
#include "estimator.h"
#include "param.h"
#include <math.h>

#define ANTENNA_OFFSET 154.6   // In meter
#define basicAddr 0xbccf851300000000
// Define: id = last_number_of_address - 5
static uint8_t selfID;
static locoAddress_t selfAddress;
static const uint64_t antennaDelay = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

static lpsRangeoptions rangeOptions = {
.userRequestedrangemode = rangemode_vanilla,
.currentrangemode = rangemode_vanilla
};

distanceMeasurement_t dist;

typedef struct {
  uint16_t distance[NumUWB];
  float stdev[1];
  float vx[NumUWB];
  float vy[NumUWB];
  float vz[NumUWB];
  float kx[NumUWB];
  float ky[NumUWB];
  float kz[NumUWB];
  float gz[NumUWB];
  float h[NumUWB];
  bool refresh[NumUWB];
  bool keep_flying;
} swarmInfo_t;
static swarmInfo_t state;
point_t estk;
point_t estpos;
//float currenttwr; 
// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static bool rangingOk;

// Communication logic between each UWB
static bool current_mode_trans;
static uint8_t current_receiveID;

 static bool checkTurn;
 static uint32_t checkTurnTick = 0;

// Median filter for distance ranging (size=3)
typedef struct {
  uint16_t distance_history[3];
  uint8_t index_inserting;
} median_data_t;
static median_data_t median_data[NumUWB];

static uint16_t median_filter_3(uint16_t *data)
{
  uint16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2])){
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if((data[1] <= data[0]) && (data[1] <= data[2])){
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else{
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (antennaDelay / 2);

  if(current_mode_trans){
    switch (txPacket.payload[0]) {
      case LPS_TWR_POLL:
        poll_tx = departure;
        break;
      case LPS_TWR_FINAL:
        final_tx = departure;
        break;
      case LPS_TWR_REPORT+1:
      
        if( (current_receiveID == 0) || (current_receiveID-1 == selfID) ){
          // current_receiveID = current_receiveID;
          current_mode_trans = false;
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, 10000);
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
          checkTurn = true;
          checkTurnTick = xTaskGetTickCount();
        }else{
          current_receiveID = current_receiveID - 1;
        }
        break;
    }
  }else{
    switch (txPacket.payload[0]) {
      case LPS_TWR_ANSWER:
        answer_tx = departure;
        break;
      case LPS_TWR_REPORT:
        break;
    }
  }
}

static void rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  if (rxPacket.destAddress != selfAddress) {
    if(current_mode_trans){
       current_mode_trans = false;
      dwIdle(dev);
      dwSetReceiveWaitTimeout(dev, 10000);
    }
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if(current_mode_trans){
    switch(rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_ANSWER:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        answer_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT:
      {
        lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
        double tround1, treply1, treply2, tround2, tprop_ctn, tprop;
        memcpy(&poll_rx, &report->pollRx, 8);
        memcpy(&answer_tx, &report->answerTx, 8);
        memcpy(&final_rx, &report->finalRx, 8);
        tround1 = answer_rx.low32 - poll_tx.low32;
        treply1 = answer_tx.low32 - poll_rx.low32;
        tround2 = final_rx.low32 - answer_tx.low32;
        treply2 = final_tx.low32 - answer_rx.low32;
        tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);
        tprop = tprop_ctn / LOCODECK_TS_FREQ;
        uint16_t calcDist = (uint16_t)(1000 * (SPEED_OF_LIGHT * tprop));
        if(calcDist!=0){
          uint16_t medianDist = median_filter_3(median_data[current_receiveID].distance_history);
          if (ABS(medianDist-calcDist)>500)
            state.distance[current_receiveID] = medianDist;
          else
            state.distance[current_receiveID] = calcDist;
          median_data[current_receiveID].index_inserting++;
          if(median_data[current_receiveID].index_inserting==3)
            median_data[current_receiveID].index_inserting = 0;
          median_data[current_receiveID].distance_history[median_data[current_receiveID].index_inserting] = calcDist;        
          rangingOk = true;
          
          state.vx[current_receiveID] = report->selfVx;
          state.vy[current_receiveID] = report->selfVy;
          state.vz[current_receiveID] = report->selfVz;
          state.kx[current_receiveID] = report->selfKx;
          state.ky[current_receiveID] = report->selfKy;
          state.kz[current_receiveID] = report->selfKz;
          state.gz[current_receiveID] = report->selfGz;
          state.h[current_receiveID]  = report->selfh;

         // if(current_receiveID<NumUWB){

          
          float div = 1000;
          dist.distance = state.distance[current_receiveID]/div;
          
          
          dist.x = state.vx[current_receiveID];
          dist.y = state.vy[current_receiveID];
          dist.z = state.vz[current_receiveID];


         
         if (rangeOptions.userRequestedrangemode != rangeOptions.currentrangemode) {
          rangeOptions.currentrangemode = rangeOptions.userRequestedrangemode;
          DEBUG_PRINT("set swarm ranging mode:  %d\n", rangeOptions.currentrangemode);
         }

         // our vanilla ranging scheme
         if(rangeOptions.currentrangemode == rangemode_vanilla){
          float div = 1000;
          dist.distance = state.distance[current_receiveID]/div;
         dist.stdDev = 0.16;
         state.stdev[1] = 0.16; 
         //DEBUG_PRINT("stdev:  %d\n", (int)(dist.stdDev*1000));
         }

        // our covariance updated ranging scheme
        if(rangeOptions.currentrangemode == rangemode_CU){
         float div = 1000;
          dist.distance = state.distance[current_receiveID]/div;
          point_t estpos;
          estimatorKalmanGetEstimatedPos(&estpos);
          float theta = atanf(estpos.y-state.vy[current_receiveID]/estpos.x-state.vx[current_receiveID]);
          float std = sqrtf(0.16f*0.16f+cosf(theta)* cosf(theta)* state.kx[current_receiveID] + sinf(theta)* sinf(theta)*(state.ky[current_receiveID]) +2*(state.kz[current_receiveID])*sinf(theta)*cosf(theta)); 
          
          if(isnan(std)){
            std = 0.16f;
           // DEBUG_PRINT("nan std ");
          }
         
          
          dist.stdDev = std;
          state.stdev[1] = std; 
        //  DEBUG_PRINT("stdev:  %d\n", (int)(dist.stdDev*1000));
         }
        if(rangeOptions.currentrangemode == rangemode_CI_CU){
           
          estimatorKalmanGetEstimatedPos(&estpos);
          estimatorKalmanGetEstimatedK(&estk);
          float theta = atanf((estpos.y-state.vy[current_receiveID])/(estpos.x-state.vx[current_receiveID]));

          float var1 = (0.16f*0.16f)+(cosf(theta)* cosf(theta)* state.kx[current_receiveID] + sinf(theta)* sinf(theta)*(state.ky[current_receiveID]) +2*(state.kz[current_receiveID])*sinf(theta)*cosf(theta)); 
          float var2 = (0.16f*0.16f)+(cosf(theta)* cosf(theta)* state.kx[current_receiveID] + sinf(theta)* sinf(theta)*(state.ky[current_receiveID]) +2*(state.kz[current_receiveID])*sinf(theta)*cosf(theta))+(cosf(theta)* cosf(theta)*(estk.x) + sinf(theta)* sinf(theta)*(estk.y) +2*(estk.z)*sinf(theta)*cosf(theta)); 
          float div = 1000;
          float dist1 = state.distance[current_receiveID]/div;
          float dist2 = sqrtf(((state.vx[current_receiveID]-estpos.x)*(state.vx[current_receiveID]-estpos.x))+((state.vy[current_receiveID]-estpos.y)*(state.vy[current_receiveID]-estpos.y)));
          
          float v1 = 1/var1; 
          float v2 = 1/var2; 
          float omega = (abs(v1+v2)-v2+v1)/(2*abs(v1+v2));
           
          float std = sqrtf(1/((omega/var1)+((1-omega)/var2)));
          float dista = std*(((omega)*dist1/var1)+(1-omega)*(dist2)/var2);
          
          if(abs(abs(dista)-abs(dist1))>2){
          dista = dist1;

          }

          if(std > 10){
          std = 0.16f;
          dista = state.distance[current_receiveID]/div; 

          }
          if(isnan(std)){
            std = 0.16f;
            dista = state.distance[current_receiveID]/div;
          }
        
          dist.stdDev =  std;
          dist.distance = dista;
          state.stdev[1] = std; 
          DEBUG_PRINT("dist:  %d\n", (int)(dist1*1000));
          DEBUG_PRINT("dista:  %d\n", (int)(dist2*1000));
         }

         if(rangeOptions.currentrangemode != rangemode_disabled){
         //DEBUG_PRINT("distance quad %d, is %d\n",current_receiveID, (int)(dist.distance*1000));
           // DEBUG_PRINT("original dist quad %d\n", (int)(dist.stdDev*1000));

            estimatorEnqueueDistance(&dist);
         }
        //  }
        //if(dist.distance!= 0.0f){
          
        //}
       
        //currenttwr = dist.distance;       
          if(current_receiveID==0)
            state.keep_flying = report->keep_flying;
          state.refresh[current_receiveID] = true;
        }

        lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(txPacket.payload+2);
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT+1;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        report2->reciprocalDistance = calcDist;
        estimatorKalmanGetSwarmInfo(&report2->selfVx, &report2->selfVy, &report2->selfVz, &report2->selfKx,&report2->selfKy,&report2->selfKz, &report2->selfGz, &report2->selfh);
        //report->selfVx = 0.3;
        //report->selfVy = 0.0;
        //report->selfVz = 0.0;

        report2->keep_flying = state.keep_flying;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
    }
  }else{
    switch(rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_POLL:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        poll_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_FINAL:
      {
        lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(txPacket.payload+2);
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        final_rx = arival;
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        memcpy(&report->pollRx, &poll_rx, 8);
        memcpy(&report->answerTx, &answer_tx, 8);
        memcpy(&report->finalRx, &final_rx, 8);
        estimatorKalmanGetSwarmInfo(&report->selfVx, &report->selfVy, &report->selfVz, &report->selfKx, &report->selfKy, &report->selfKz, &report->selfGz, &report->selfh);

        //report->selfVx = 0.3;
        //report->selfVy = 0.0;
        //report->selfVz = 0.0;


        report->keep_flying = state.keep_flying;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case (LPS_TWR_REPORT+1):
      {
        lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
        uint8_t rangingID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if((report2->reciprocalDistance)!=0){
          // received distance has large noise
          uint16_t calcDist = report2->reciprocalDistance;
          uint16_t medianDist = median_filter_3(median_data[rangingID].distance_history);
          if (ABS(medianDist-calcDist)>500)
            state.distance[rangingID] = medianDist;
          else
            state.distance[rangingID] = calcDist;
          median_data[rangingID].index_inserting++;
          if(median_data[rangingID].index_inserting==3)
            median_data[rangingID].index_inserting = 0;
          median_data[rangingID].distance_history[median_data[rangingID].index_inserting] = calcDist; 
          state.vx[rangingID] = report2->selfVx;
          state.vy[rangingID] = report2->selfVy;
          state.vz[rangingID] = report2->selfVz;
          state.kx[rangingID] = report2->selfKx;
          state.ky[rangingID] = report2->selfKy;
          state.kz[rangingID] = report2->selfKz;
          state.gz[rangingID] = report2->selfGz;
          state.h[rangingID]  = report2->selfh;
          
         // if(rangingID < NumUWB){

          
          dist.x = state.vx[rangingID];
          dist.y = state.vy[rangingID];
          dist.z = state.vz[rangingID];


         
         if (rangeOptions.userRequestedrangemode != rangeOptions.currentrangemode) {
          rangeOptions.currentrangemode = rangeOptions.userRequestedrangemode;
          DEBUG_PRINT("set swarm ranging mode:  %d\n", rangeOptions.currentrangemode);
         }

         // our vanilla ranging scheme
         if(rangeOptions.currentrangemode == rangemode_vanilla){
          float div = 1000;
          dist.distance = state.distance[rangingID]/div;
          dist.stdDev = 0.16f;
          state.stdev[1] = 0.16f; 
         // DEBUG_PRINT("stdev:  %d\n", (int)(dist.stdDev*1000));
         }

        // our covariance updated ranging scheme
        if(rangeOptions.currentrangemode == rangemode_CU){
          float div = 1000;
          dist.distance = state.distance[rangingID]/div;
          point_t estpos;
          estimatorKalmanGetEstimatedPos(&estpos);
          float theta = atanf(estpos.y-state.vy[rangingID]/estpos.x-state.vx[rangingID]);
          float std = sqrtf(0.16f*0.16f+cosf(theta)* cosf(theta)* state.kx[rangingID] + sinf(theta)* sinf(theta)*(state.ky[rangingID]) +2*(state.kz[rangingID])*sinf(theta)*cosf(theta)); 
          
          if(isnan(std)){
            std = 0.16f;
          //  DEBUG_PRINT("nan std ");
          }
         
          
          dist.stdDev = std;
          state.stdev[0] = std; 
          //DEBUG_PRINT("stdev:  %d\n", (int)(dist.stdDev*1000));
         }
        if(rangeOptions.currentrangemode == rangemode_CI_CU){
          
          estimatorKalmanGetEstimatedPos(&estpos);
          estimatorKalmanGetEstimatedK(&estk);
          float theta = atanf((estpos.y-state.vy[rangingID])/(estpos.x-state.vx[rangingID]));

          float var1 = 0.16f*0.16f+(cosf(theta)* cosf(theta)* state.kx[rangingID] + sinf(theta)* sinf(theta)*(state.ky[rangingID]) +2*(state.kz[rangingID])*sinf(theta)*cosf(theta)); 
          float var2 = 0.16f*0.16f+(cosf(theta)* cosf(theta)* state.kx[rangingID] + sinf(theta)* sinf(theta)*(state.ky[rangingID]) +2*(state.kz[rangingID])*sinf(theta)*cosf(theta))+(cosf(theta)* cosf(theta)*(estk.x) + sinf(theta)* sinf(theta)*(estk.y) +2*(estk.z)*sinf(theta)*cosf(theta)); 
          float div = 1000;
          float dist1 = state.distance[rangingID]/div;
          float dist2 = sqrtf(((state.vx[rangingID]-estpos.x)*(state.vx[rangingID]-estpos.x))+((state.vy[rangingID]-estpos.y)*(state.vy[rangingID]-estpos.y)));

          float v1 = 1/var1; 
          float v2 = 1/var2; 
          float omega = (abs(v1+v2)-v2+v1)/(2*abs(v1+v2));
           
          float std = sqrtf(1/((omega/var1)+((1-omega)/var2)));
          float dista = std*(((omega)*dist1/var1)+(1-omega)*(dist2)/var2);

          if(abs(abs(dista)-abs(dist1))>2){
          dista = dist1;

          }
          if(std > 10){
          std = 0.16f;
          dista = state.distance[current_receiveID]/div; 
          }

          if(isnan(std)){
            std = 0.16f;
            dista = state.distance[rangingID]/div;
            //DEBUG_PRINT("nan std ");
          }
       
          dist.stdDev =  std;
          state.stdev[0] = std; 
          dist.distance = dista;
          DEBUG_PRINT("dist:  %d\n", (int)(dist1*1000));
          DEBUG_PRINT("dista:  %d\n", (int)(dist2*1000));
          //DEBUG_PRINT("stdev:  %d\n", (int)(estk.y*1000));

         }

         if(rangeOptions.currentrangemode != rangemode_disabled){
        
         //DEBUG_PRINT("distance quad %d, is %d\n",rangingID, (int)(dist.distance*1000));
         //state.stdev = std; 
        estimatorEnqueueDistance(&dist);
         }
        //}
     //  }
        //currenttwr = dist.distance;  
          if(rangingID==0)
            state.keep_flying = report2->keep_flying;
          state.refresh[rangingID] = true;
          
        }
        rangingOk = true;
        
        uint8_t fromID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if( selfID == fromID + 1 || selfID == 0 ){
          current_mode_trans = true;
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, 1000);
          if(selfID == NumUWB-1)
            current_receiveID = 0;
          else
            current_receiveID = NumUWB - 1;

          if(NumUWB>2)
            if(selfID == 0)
              current_receiveID = NumUWB - 2; // immediate problem
            
          txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
          txPacket.payload[LPS_TWR_SEQ] = 0;
          txPacket.sourceAddress = selfAddress;
          txPacket.destAddress = basicAddr + current_receiveID;
          dwNewTransmit(dev);
          dwSetDefaults(dev);
          dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
          dwWaitForResponse(dev, true);
          dwStartTransmit(dev);
        }else{
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
         }
        break;
      }
    }
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
       checkTurn = false;
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
    case eventReceiveTimeout:
    case eventReceiveFailed:
      if (current_mode_trans==true)
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }else
      {
        
        if(xTaskGetTickCount() > checkTurnTick + 20) // > 20ms
        {
          if(checkTurn == true){
            current_mode_trans = true;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = selfAddress;
            txPacket.destAddress = basicAddr + current_receiveID;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
            checkTurn = false;
            break;
          }
        }
        
        dwNewReceive(dev);
	      dwSetDefaults(dev);
        dwStartReceive(dev);
      }     
      break;
    default:
      configASSERT(false);
  }
  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 5);
  selfAddress = basicAddr + selfID;

  // Communication logic between each UWB
  if(selfID==0)
  {
    current_receiveID = NumUWB-1;
    current_mode_trans = true;
    dwSetReceiveWaitTimeout(dev, 1000);
  }
  else 
  {
   // current_receiveID = 0;
    current_mode_trans = false;
    dwSetReceiveWaitTimeout(dev, 10000);
  }

  for (int i = 0; i < NumUWB; i++) {
    median_data[i].index_inserting = 0;
    state.refresh[i] = false;
  }

  state.keep_flying = false;
   checkTurn = false;
  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}


static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < NumUWB-1)
    return true;
  else
    return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < NumUWB-1; i++) {
    unorderedAnchorList[i] = i;
  }
  return NumUWB-1;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;
  for (int i = 0; i < NumUWB-1; i++) {
      unorderedAnchorList[count] = i;
      count++;
  }
  return count;
}

bool twrGetSwarmInfo(int robNum, uint16_t* range, float* vx, float* vy, float* vz, float* gyroZ, float* height) {
  if(state.refresh[robNum]==true) {
    state.refresh[robNum] = false;
    *range = state.distance[robNum];
    *vx = state.vx[robNum];
    *vy = state.vy[robNum];
    *vz = state.vz[robNum];
    *gyroZ = state.gz[robNum];
    *height = state.h[robNum];
    return(true);
  }else {
    return(false);
  }
}

bool command_share(int RobIDfromControl, bool keep_flying) {
  if(RobIDfromControl==0){
    state.keep_flying = keep_flying;
    return keep_flying;
  }else{
    return state.keep_flying;
  }
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, distance0, &state.distance[0])
LOG_ADD(LOG_UINT16, distance1, &state.distance[1])
LOG_ADD(LOG_UINT16, distance2, &state.distance[2])
LOG_ADD(LOG_UINT16, distance3, &state.distance[3])
LOG_ADD(LOG_UINT16, distance4, &state.distance[4])
LOG_ADD(LOG_FLOAT, rangedev, &dist.stdDev)
LOG_GROUP_STOP(ranging)


LOG_GROUP_START(ranging_mode)
LOG_ADD_CORE(LOG_UINT8, range_modus,&rangeOptions.currentrangemode)
LOG_GROUP_STOP(ranging_mode)

PARAM_GROUP_START(ranging_mode)
PARAM_ADD_CORE(PARAM_UINT8, range_modus, &rangeOptions.userRequestedrangemode)
PARAM_GROUP_STOP(ranging_mode)