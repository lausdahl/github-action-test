#include "co-sim.hxx"
#include <stdint.h>
#include <string>
#include "DataWriter.h"
#include "DataWriterConfig.h"
#include "SimFmi2.h"
#include "Logger.h"
#include "SimMath.h"
void simulate(const char* __runtimeConfigPath)
{
	double START_TIME = 0.0;
	double END_TIME = 10.0;
	double STEP_SIZE = 0.1;
	DataWriter dataWriter = load_DataWriter(__runtimeConfigPath);
	Logger logger = load_Logger();
	Math math = load_Math();
	FMI2 tankcontroller = load_FMI2("{8c4e810f-3df3-4a00-8276-176fa3c9f000}", "watertankcontroller-c.fmu");
	FMI2 SingleWatertank = load_FMI2("{cfc65592-9ece-4563-9705-1581b6e7071c}", "singlewatertank-20sim.fmu");
	FMI2Component crtlInstance = tankcontroller->instantiate("crtlInstance", false, false);
	FMI2Component wtInstance = SingleWatertank->instantiate("wtInstance", false, false);
	int global_execution_continue = true;
	int status = 0;
	while(global_execution_continue)
	{
		double fixed_end_time = 0.0;
		fixed_end_time = END_TIME;
		double wtinstanceRealShare[1] = {};
		int crtlinstanceBoolShare[1] = {};
		int crtlinstanceBoolIo[4] = {};
		double crtlinstanceRealIo[4] = {};
		unsigned int crtlinstanceUintVref[4] = {};
		double wtinstanceRealIo[22] = {};
		unsigned int wtinstanceUintVref[22] = {};
		status = wtInstance->fmu->setupExperiment(wtInstance->comp, true, 0.1, START_TIME, true, END_TIME);
		status = crtlInstance->fmu->setupExperiment(crtlInstance->comp, true, 0.1, START_TIME, true, END_TIME);
		wtinstanceUintVref[0] = 0;
		wtinstanceRealIo[0] = 9.0;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 1;
		wtinstanceRealIo[0] = 1.0;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 2;
		wtinstanceRealIo[0] = 1.0;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 3;
		wtinstanceRealIo[0] = 9.81;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 4;
		wtinstanceRealIo[0] = 1.0;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 5;
		wtinstanceRealIo[0] = 0.0;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 6;
		wtinstanceRealIo[0] = 0.0;
		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		crtlinstanceUintVref[0] = 0;
		crtlinstanceRealIo[0] = 2.0;
		status = crtlInstance->fmu->setReal(crtlInstance->comp, crtlinstanceUintVref, 1, crtlinstanceRealIo);
		crtlinstanceUintVref[0] = 1;
		crtlinstanceRealIo[0] = 1.0;
		status = crtlInstance->fmu->setReal(crtlInstance->comp, crtlinstanceUintVref, 1, crtlinstanceRealIo);
		status = wtInstance->fmu->enterInitializationMode(wtInstance->comp);
		status = crtlInstance->fmu->enterInitializationMode(crtlInstance->comp);
		crtlinstanceUintVref[0] = 4;
		status = crtlInstance->fmu->getBoolean(crtlInstance->comp, crtlinstanceUintVref, 1, crtlinstanceBoolIo);
		crtlinstanceBoolShare[0] = crtlinstanceBoolIo[0];
		wtinstanceUintVref[0] = 16;
		if(crtlinstanceBoolShare[0])
		{
			wtinstanceRealIo[0] = 1.0;
		}
		else
		{
			wtinstanceRealIo[0] = 0.0;
		}

		status = wtInstance->fmu->setReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceUintVref[0] = 17;
		status = wtInstance->fmu->getReal(wtInstance->comp, wtinstanceUintVref, 1, wtinstanceRealIo);
		wtinstanceRealShare[0] = wtinstanceRealIo[0];
		crtlinstanceUintVref[0] = 3;
		crtlinstanceRealIo[0] = wtinstanceRealShare[0];
		status = crtlInstance->fmu->setReal(crtlInstance->comp, crtlinstanceUintVref, 1, crtlinstanceRealIo);
		status = wtInstance->fmu->exitInitializationMode(wtInstance->comp);
		status = crtlInstance->fmu->exitInitializationMode(crtlInstance->comp);
		break;
	}

	if(global_execution_continue)
	{
		{
			double end = END_TIME - STEP_SIZE;
			double time = START_TIME;
			double fix_stepSize = 0.0;
			double fix_recoveryStepSize = 0.0;
			int fix_recovering = false;
			int fix_global_status = false;
			int fix_comp_index = 0;
			double wtInstanceROut[1] = {};
			int crtlInstanceBOut[1] = {};
			unsigned int wtInstanceVrefROut[1] = {17};
			unsigned int crtlInstanceVrefBOut[1] = {4};
			double wtInstanceRIn[1] = {};
			double crtlInstanceRIn[1] = {};
			unsigned int wtInstanceVrefRIn[1] = {16};
			unsigned int crtlInstanceVrefRIn[1] = {3};
			int fix_status[2] = {0, 0, };
			fix_status[0] = wtInstance->fmu->getReal(wtInstance->comp, wtInstanceVrefROut, 1, wtInstanceROut);
			if(fix_status[0] == 3 || fix_status[0] == 4)
			{
				logger->log(4, "get failed %d ", fix_status[fix_comp_index]);
				global_execution_continue = false;
			}

			fix_status[1] = crtlInstance->fmu->getBoolean(crtlInstance->comp, crtlInstanceVrefBOut, 1, crtlInstanceBOut);
			if(fix_status[1] == 3 || fix_status[1] == 4)
			{
				logger->log(4, "get failed %d ", fix_status[fix_comp_index]);
				global_execution_continue = false;
			}

			const char* data_headers[2] = {"{x1}.crtlInstance.valve", "{x2}.wtInstance.level"};
			DataWriterConfig dataWriter_configuration = dataWriter->writeHeader(2, data_headers);
			dataWriter->writeDataPoint("ir", dataWriter_configuration, time, crtlInstanceBOut[0], wtInstanceROut[0]);
			while(global_execution_continue && time <= end)
			{
				if(fix_recovering)
				{
					fix_stepSize = fix_recoveryStepSize;
					fix_recovering = false;
				}
				else
				{
					fix_stepSize = STEP_SIZE;
				}

				crtlInstanceRIn[0] = wtInstanceROut[0];
				if(crtlInstanceBOut[0])
				{
					wtInstanceRIn[0] = 1.0;
				}
				else
				{
					wtInstanceRIn[0] = 0.0;
				}

				fix_status[0] = wtInstance->fmu->setReal(wtInstance->comp, wtInstanceVrefRIn, 1, wtInstanceRIn);
				if(fix_status[0] == 3 || fix_status[0] == 4)
				{
					logger->log(4, "set failed %d ", fix_status[fix_comp_index]);
					global_execution_continue = false;
					break;
				}

				fix_status[1] = crtlInstance->fmu->setReal(crtlInstance->comp, crtlInstanceVrefRIn, 1, crtlInstanceRIn);
				if(fix_status[1] == 3 || fix_status[1] == 4)
				{
					logger->log(4, "set failed %d ", fix_status[fix_comp_index]);
					global_execution_continue = false;
					break;
				}

				fix_status[0] = wtInstance->fmu->doStep(wtInstance->comp, time, fix_stepSize, true);
				fix_status[1] = crtlInstance->fmu->doStep(crtlInstance->comp, time, fix_stepSize, true);
				fix_global_status = true;
				fix_comp_index = 0;
				while(fix_comp_index < 2)
				{
					if(fix_status[fix_comp_index] != 0)
					{
						fix_global_status = false;
						logger->log(4, "doStep failed for %d - status code ", fix_status[fix_comp_index]);
						break;
					}

					fix_comp_index = fix_comp_index + 1;
				}

				if(!fix_global_status)
				{
					logger->log(4, "Deviating from normal execution. Handling exceptions %d", 0);
					fix_global_status = true;
					fix_comp_index = 0;
					int discardObserved = false;
					while(fix_comp_index < 2)
					{
						logger->log(4, "Fmu index %d, status code %d", fix_comp_index, fix_status[fix_comp_index]);
						if(fix_status[fix_comp_index] != 0)
						{
							fix_global_status = false;
							if(fix_status[fix_comp_index] == 5)
							{
								logger->log(4, "doStep failed for %d PENDING not supported- status code ", fix_status[fix_comp_index]);
							}
							else
							{
								if(fix_status[fix_comp_index] == 3 || fix_status[fix_comp_index] == 4)
								{
									logger->log(4, "doStep failed for %d - status code ", fix_status[fix_comp_index]);
								}

							}

							if(fix_status[fix_comp_index] == 2)
							{
								logger->log(2, "Instance discarding %d", fix_comp_index);
								discardObserved = true;
							}

							global_execution_continue = false;
							break;
						}

						fix_comp_index = fix_comp_index + 1;
					}

					if(!global_execution_continue)
					{
						break;
					}

				}

				if(global_execution_continue && !fix_recovering)
				{
					fix_status[0] = wtInstance->fmu->getReal(wtInstance->comp, wtInstanceVrefROut, 1, wtInstanceROut);
					if(fix_status[0] == 3 || fix_status[0] == 4)
					{
						logger->log(4, "get failed %d ", fix_status[fix_comp_index]);
						global_execution_continue = false;
						break;
					}

					fix_status[1] = crtlInstance->fmu->getBoolean(crtlInstance->comp, crtlInstanceVrefBOut, 1, crtlInstanceBOut);
					if(fix_status[1] == 3 || fix_status[1] == 4)
					{
						logger->log(4, "get failed %d ", fix_status[fix_comp_index]);
						global_execution_continue = false;
						break;
					}

					time = time + fix_stepSize;
					dataWriter->writeDataPoint("ir", dataWriter_configuration, time, crtlInstanceBOut[0], wtInstanceROut[0]);
				}

			}

			wtInstance->fmu->terminate(wtInstance->comp);
			crtlInstance->fmu->terminate(crtlInstance->comp);
			dataWriter->close();
		}

	}

	tankcontroller->freeInstance(crtlInstance->comp);
	SingleWatertank->freeInstance(wtInstance->comp);
	delete tankcontroller;
	tankcontroller = nullptr;
	delete SingleWatertank;
	SingleWatertank = nullptr;
	delete dataWriter;
	dataWriter = nullptr;
	delete logger;
	logger = nullptr;
	delete math;
	math = nullptr;
}
