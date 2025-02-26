#ifndef MSGQUEUE_H
#define MSGQUEUE_H

#define D_QUE_MAXMSG_NUM (20)

#define D_QUE_NAME_SIMULATION "/Simulation"
#define D_QUE_NAME_HMICTRL "/HmiCtrl"
#define D_QUE_NAME_AUTOTEST "/AutoTest"
#define D_QUE_NAME_Evaluation "/Evaluation"
#define D_QUE_NAME_VehicleDynamics "/VehicleDynamics"
#define D_QUE_NAME_ScenarioFileManager "/ScenarioFileManager"
#define D_QUE_NAME_Generalize "/Generalize"
#define D_QUE_NAME_LOG "/Log"
#define D_QUE_NAME_EXCEEDREALTIME "/ExceedRealTime" //zxl 2021.09.01 用于超实时功能
#define D_QUE_NAME_SceneEditor "/SceneEditor"
#define D_QUE_NAME_COSIMUSOCKET "/CosimuSocket"

#define D_QUE_NAME_ScenarioCapture "/ScenarioCapture"


#ifndef MACRO_SYSTEMCTROL    //define system ctrol macro
#define MACRO_SYSTEMCTROL
#endif

#ifdef MACRO_SYSTEMCTROL
#define D_QUE_NAME_SystemCtrol "/SystemCtrol"
#define D_QUE_NAME_Vtraffic "/Vtraffic"
#endif

/* Who */
#define D_QUE_WHO_MAIN       (0x00000100)
#define D_QUE_WHO_SIMULATION (0x00000200)
#define D_QUE_WHO_HMICTRL    (0x00000400)
#define D_QUE_WHO_AUTOTEST   (0x00000800)
#define D_QUE_WHO_Evaluation (0x00001000)
#define D_QUE_WHO_ScenarioFileManager (0x00002000)
#define D_QUE_WHO_Generalize (0x00004000)


#ifdef MACRO_SYSTEMCTROL
#define D_QUE_WHO_SystemCtrol    (0x00008000)
#define D_QUE_WHO_Vtraffic	     (0x00010000)
#endif


/* ERR CODE */
#define D_QUE_WHO_ERR_TERMINATOR (0x000000FF)

#define D_QUE_PARAM_DATA_SIZE (8188)


typedef struct s_que_
{
	unsigned int u4Who;					   /* :  Sender = Data producer */
	uint8_t u1Data[D_QUE_PARAM_DATA_SIZE]; /* :  parameters */
} S_QUE;

#ifdef MACRO_SYSTEMCTROL

#define D_QUE_PARAM_COMMAND_SIZE (2)
typedef struct s_que_systemCt_
{
	unsigned int u4Who;					   /* :  Sender = Data producer */
	uint8_t u1Data[D_QUE_PARAM_COMMAND_SIZE]; /* :  parameters just command */
} S_QUE_SYSTEMCT;

#endif

#endif /* MSGQUEUE_H */
