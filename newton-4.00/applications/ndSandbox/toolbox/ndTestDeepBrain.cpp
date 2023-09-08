/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndTestDeepBrain.h"

static void ThreeLayersTwoInputsTwoOutputs()
{
	ndBrain brain;
	ndInt32 hiddenNeurons = 32;
#if 0
	//ndBrainLayerLinearActivated* const inputLayer = new ndBrainLayerLinearActivated(2, hiddenNeurons, m_tanh);
	//ndBrainLayerLinearActivated* const hiddenLayer0 = new ndBrainLayerLinearActivated(inputLayer->GetOuputSize(), hiddenNeurons, m_tanh);
	//ndBrainLayerLinearActivated* const hiddenLayer1 = new ndBrainLayerLinearActivated(hiddenLayer0->GetOuputSize(), hiddenNeurons, m_tanh);
	//ndBrainLayerLinearActivated* const ouputLayer = new ndBrainLayerLinearActivated(hiddenLayer1->GetOuputSize(), 2, m_sigmoid);
	//
	//brain.AddLayer(inputLayer);
	//brain.AddLayer(hiddenLayer0);
	//brain.AddLayer(hiddenLayer1);
	//brain.AddLayer(ouputLayer);
	//brain.InitWeightsXavierMethod();
#else
	hiddenNeurons = 2;
	ndBrainLayer* const inputLayer = new ndBrainLayerLineal(2, hiddenNeurons);
	ndBrainLayer* const activation0 = new ndBrainLayerSigmoidActivation(hiddenNeurons);

	brain.AddLayer(inputLayer);
	brain.AddLayer(activation0);

#endif
	brain.InitWeightsXavierMethod();
	
	ndInt32 samples = 2000;
	ndBrainMatrix inputBatch(samples, 2);
	ndBrainMatrix groundTruth(samples, 2);
	for (ndInt32 i = 0; i < samples; i++)
	{
		inputBatch[i][0] = ndClamp (ndReal(ndGaussianRandom(0.5f, 0.25f)), ndReal(0.0f), ndReal(1.0f));
		inputBatch[i][1] = ndClamp (ndReal(ndGaussianRandom(0.5f, 0.25f)), ndReal(0.0f), ndReal(1.0f));
	
		groundTruth[i][0] = ((inputBatch[i][0] >= ndReal(0.5f)) && (inputBatch[i][1] >= ndReal(0.5f))) ? ndReal(1.0f) : ndReal(0.0f);
		groundTruth[i][1] = ((inputBatch[i][0] >= ndReal(0.5f)) || (inputBatch[i][1] >= ndReal(0.5f))) ? ndReal(1.0f) : ndReal(0.0f);
	}
	
	const ndInt32 bashSize = 64;

	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> trainers;

	ndBrainThreadPool threads;
	//threads.SetThreadCount(4);
	threads.SetThreadCount(1);
	for (ndInt32 i = 0; i < threads.GetThreadCount(); ++i)
	{
		trainers.PushBack(new ndBrainTrainer(&brain));
	}

	ndInt32 randomeSelection[bashSize];

	auto UpdateTrainer = ndMakeObject::ndFunction([&trainers, &randomeSelection, &inputBatch, &groundTruth, bashSize](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*trainers[threadIndex]);
		trainer.ClearGradientsAcc();
		ndBrainLeastSquareErrorLoss loss(trainer.GetBrain()->GetOutputSize());
		const ndStartEnd startEnd(bashSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = randomeSelection[i];
			const ndBrainVector& input = inputBatch[index];
			//loss.SetTruth(groundTruth[index]);
			//trainer.BackPropagate(input, loss);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([&trainers](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*trainers[0]);
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer& srcTrainer = *(*trainers[i]);
			trainer.AcculumateGradients(srcTrainer, threadIndex, threadCount);
		}
	});

	for (ndInt32 i = 0; i < 20000; ++i)
	{
		for (ndInt32 j = 0; j < bashSize; ++j)
		{
			randomeSelection[j] = ndInt32 (ndRandInt() % samples);
		}
		threads.ParallelExecute(UpdateTrainer);
		//threads.ParallelExecute(AccumulateWeight);
		//trainers[0]->UpdateWeights(ndReal(1.0e-2f), bashSize);
	}
	
	ndBrainVector truth;
	ndBrainVector input;
	ndBrainVector output;
	
	input.SetCount(brain.GetInputSize());
	truth.SetCount(brain.GetOutputSize());
	output.SetCount(brain.GetOutputSize());
	
	ndInt32 failCount = 0;
	ndInt32 testCount = 200;
	for (ndInt32 i = 0; i < testCount; ++i)
	{
		input[0] = ndReal(ndGaussianRandom(0.5f, 0.25f));
		input[1] = ndReal(ndGaussianRandom(0.5f, 0.25f));
		truth[0] = ((input[0] >= ndReal(0.5f)) && (input[1] >= ndReal(0.5f))) ? ndReal(1.0f) : ndReal(0.0f);
		truth[1] = ((input[0] >= ndReal(0.5f)) || (input[1] >= ndReal(0.5f))) ? ndReal(1.0f) : ndReal(0.0f);
	
		brain.MakePrediction(input, output);
	
		bool predicted = true;
		for (ndInt32 j = 0; j < output.GetCount(); ++j)
		{
			bool trueBit = truth[j] >= ndReal(0.5f);
			bool predictBit = output[j] >= ndReal(0.5f);
			predicted = predicted && (predictBit == trueBit);
		}
	
		if (!predicted)
		{
			failCount++;
		}
	}
	
	ndExpandTraceMessage("%s\n", "boolean logic");
	ndExpandTraceMessage("num_right: %d  out of %d\n", testCount - failCount, testCount);
	ndExpandTraceMessage("num_wrong: %d  out of %d\n", failCount, testCount);
	ndExpandTraceMessage("success rate %g%%\n", (ndFloat32)(testCount - failCount) * 100.0f / (ndFloat32)testCount);
}

static ndBrainMatrix* LoadMnistLabelData(const char* const filename)
{
	ndBrainMatrix* labelData = nullptr;

	char outPathName[1024];
	ndGetWorkingFileName(filename, outPathName);
	FILE* fp = fopen(outPathName, "rb");
	if (fp)
	{
		// read training labels
		//[offset] [type]          [value]          [description]
		//0000     32 bit integer  0x00000801(2049) magic number(MSB first)
		//0004     32 bit integer  60000            number of items
		//0008     unsigned byte ? ? label
		//0009     unsigned byte ? ? label
		//........
		//xxxx     unsigned byte ? ? label
		//The labels values are 0 to 9.

		ndUnsigned32 magicNumber;
		ndUnsigned32 numberOfItems;
		size_t ret = 0;
		ret = fread(&magicNumber, 4, 1, fp);
		ret = fread(&numberOfItems, 4, 1, fp);
		magicNumber = ndIndian32(magicNumber);
		numberOfItems = ndIndian32(numberOfItems);

		labelData = new ndBrainMatrix(ndInt32 (numberOfItems), 10);
		labelData->Set(0.0f);
		for (ndUnsigned32 i = 0; i < numberOfItems; ++i)
		{
			ndUnsigned8 label;
			ret = fread(&label, 1, 1, fp);
			(*labelData)[ndInt32(i)][ndInt32(label)] = 1.0f;
		}
		fclose(fp);
	}
	return labelData;
}

static ndBrainMatrix* LoadMnistSampleData(const char* const filename)
{
	ndBrainMatrix* trainingDigits = nullptr;

	char outPathName[1024];
	ndGetWorkingFileName(filename, outPathName);
	FILE* const fp = fopen(outPathName, "rb");
	if (fp)
	{
		//[offset] [type]          [value]          [description]
		//0000     32 bit integer  0x00000803(2051) magic number
		//0004     32 bit integer  60000            number of images
		//0008     32 bit integer  28               number of rows
		//0012     32 bit integer  28               number of columns
		//0016     unsigned byte ? ? pixel
		//0017     unsigned byte ? ? pixel
		//........
		//xxxx     unsigned byte ? ? pixel/

		size_t ret = 0;
		ndUnsigned32 magicNumber;
		ndUnsigned32 numberOfItems;
		ndUnsigned32 digitWith;
		ndUnsigned32 digitHeight;

		ret = fread(&magicNumber, 4, 1, fp);
		ret = fread(&numberOfItems, 4, 1, fp);
		ret = fread(&digitWith, 4, 1, fp);
		ret = fread(&digitHeight, 4, 1, fp);
		magicNumber = ndIndian32(magicNumber);
		numberOfItems = ndIndian32(numberOfItems);

		digitWith = ndIndian32(digitWith);
		digitHeight = ndIndian32(digitHeight);
		trainingDigits = new ndBrainMatrix(ndInt32(numberOfItems), ndInt32(digitWith * digitHeight));
		trainingDigits->Set(0.0f);

		ndUnsigned8 data[32 * 32];
		for (ndUnsigned32 i = 0; i < numberOfItems; ++i)
		{
			ndBrainVector& image = (*trainingDigits)[ndInt32(i)];
			ret = fread(data, digitWith, digitHeight, fp);
			for (ndUnsigned32 j = 0; j < digitWith * digitHeight; j++)
			{
				image[ndInt32(j)] = ndReal(data[ndInt32(j)]) / 255.0f;
			}
		}
		fclose(fp);
	}
	return trainingDigits;
}

static void ValidateData(const char* const title, ndBrain& brain, ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
{
	ndBrainVector output;
	output.SetCount((*testLabels)[0].GetCount());

	ndInt32 failCount = 0;
	for (ndInt32 i = 0; i < testDigits->GetCount(); i++)
	{
		const ndBrainVector& input = (*testDigits)[i];
		brain.MakePrediction(input, output);

		const ndBrainVector& truth = (*testLabels)[i];

		ndInt32 index = 0;
		ndFloat32 maxProbability = 0.0f;
		for (ndInt32 j = 0; j < output.GetCount(); j++)
		{
			if (output[j] > maxProbability)
			{
				index = j;
				maxProbability = output[j];
			}
		}

		if (truth[index] < 0.5f)
		{
			failCount++;
		}
	}
	ndExpandTraceMessage("%s\n", title);
	ndExpandTraceMessage("num_right: %d  out of %d\n", testDigits->GetCount() - failCount, testDigits->GetCount());
	ndExpandTraceMessage("num_wrong: %d  out of %d\n", failCount, testDigits->GetCount());
	ndExpandTraceMessage("success rate %f%%\n", (ndFloat32)(testDigits->GetCount() - failCount) * 100.0f / (ndFloat32)testDigits->GetCount());
}

static void MnistTrainingSet()
{
	ndSharedPtr<ndBrainMatrix> trainingLabels (LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> trainingDigits (LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte"));

	class SupervisedTrainer : public ndBrainThreadPool
	{
		public:
		SupervisedTrainer(ndBrain* const brain)
			:ndBrainThreadPool()
			,m_brain(*brain)
			,m_learnRate(ndReal(0.001f))
			,m_bashBufferSize(64)
		{
			ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize/4);
threadCount = 1;

			SetThreadCount(threadCount);
			for (ndInt32 i = 0; i < GetThreadCount(); ++i)
			{
				ndBrainTrainer* const trainer = new ndBrainTrainer(&m_brain);
				trainer->SetModel(ndBrainTrainer::m_adam);
				trainer->SetRegularizer(ndReal(1.0e-4f));
				m_optimizers.PushBack(trainer);
			}
		}

		void Optimize(ndBrainMatrix* const trainingLabels, ndBrainMatrix* const trainingDigits)
		{
			ndUnsigned32 shuffleBashBuffer[1024];
			ndArray<ndUnsigned32> priorityIndexArray;
			
			auto BackPropagateBash = ndMakeObject::ndFunction([this, trainingDigits, trainingLabels, &shuffleBashBuffer](ndInt32 threadIndex, ndInt32 threadCount)
			{
				class Loss : public ndBrainLeastSquareErrorLoss
				{
					public:
					Loss(ndInt32 size)
						:ndBrainLeastSquareErrorLoss(size)
					{
					}
				
					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						ndBrainLeastSquareErrorLoss::GetLoss(output, loss);
					}
				
					const ndBrainMatrix* m_trainingLabels;
				};

				ndBrainTrainer& trainer = *(*m_optimizers[threadIndex]);
				trainer.ClearGradientsAcc();

				Loss loss(m_brain.GetOutputSize());
				const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndInt32 index = ndInt32(shuffleBashBuffer[i]);
					loss.SetTruth((*trainingLabels)[index]);
					trainer.BackPropagate((*trainingDigits)[index], loss);
				}
			});

			auto AccumulateBashWeights = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
			{
				ndBrainTrainer& trainer = *(*m_optimizers[0]);
				for (ndInt32 i = 1; i < threadCount; ++i)
				{
					ndBrainTrainer& srcTrainer = *(*m_optimizers[i]);
					trainer.AcculumateGradients(srcTrainer, threadIndex, threadCount);
				}
			});

			ndBrain bestBrain(m_brain);
			ndInt32 minFail = trainingDigits->GetCount();
			//for (ndInt32 i = 0; i < 2000000; ++i)
			for (ndInt32 i = 0; i < 10000; ++i)
			{
				ndInt32 priorityStart = ndMin(priorityIndexArray.GetCount(), 2);
				for (ndInt32 j = 0; j < priorityStart; ++j)
				{
					shuffleBashBuffer[j] = priorityIndexArray[ndInt32 (ndRandInt() % ndUnsigned32(priorityIndexArray.GetCount()))];
				}
				for (ndInt32 j = priorityStart; j < m_bashBufferSize; ++j)
				{
					shuffleBashBuffer[j] = ndRandInt() % trainingDigits->GetCount();
				}
				ndBrainThreadPool::ParallelExecute(BackPropagateBash);
				ndBrainThreadPool::ParallelExecute(AccumulateBashWeights);
				m_optimizers[0]->UpdateWeights(m_learnRate, m_bashBufferSize);

				if (i % 2000 == 0)
				{
					ndFixSizeArray<ndFixSizeArray<ndUnsigned32, 1024>, D_MAX_THREADS_COUNT> failPriorities;
					auto CrossValidate = ndMakeObject::ndFunction([this, trainingDigits, trainingLabels, &failPriorities](ndInt32 threadIndex, ndInt32 threadCount)
					{
						ndReal outputBuffer[32];
						ndBrainMemVector output(outputBuffer, m_brain.GetOutputSize());

						ndFixSizeArray<ndUnsigned32, 1024>& priorityArray = failPriorities[threadIndex];
						priorityArray.SetCount(0);
						const ndStartEnd startEnd(trainingDigits->GetCount(), threadIndex, threadCount);
						for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
						{
							if (priorityArray.GetCount() < priorityArray.GetCapacity())
							{
								const ndBrainVector& truth = (*trainingLabels)[i];
								const ndBrainVector& input = (*trainingDigits)[i];
								m_brain.MakePrediction(input, output);

								ndInt32 index = 0;
								ndFloat32 maxProbability = 0.0f;
								for (ndInt32 j = 0; j < output.GetCount(); j++)
								{
									if (output[j] > maxProbability)
									{
										index = j;
										maxProbability = output[j];
									}
								}

								if (truth[index] < 0.5f)
								{
									priorityArray.PushBack(ndUnsigned32(i));
								}
							}
						}
					});


					priorityIndexArray.SetCount(0);
					ndBrainThreadPool::ParallelExecute(CrossValidate);
					for (ndInt32 j = 0; j < GetThreadCount(); ++j)
					{
						ndFixSizeArray<ndUnsigned32, 1024>& priorityArray = failPriorities[j];
						for (ndInt32 k = priorityArray.GetCount() - 1; k; --k)
						{
							priorityIndexArray.PushBack(priorityArray[k]);
						}
					}

					//ndExpandTraceMessage("%f%%\n", (ndFloat32)(trainingDigits->GetCount() - priorityIndexArray.GetCount()) * 100.0f / (ndFloat32)trainingDigits->GetCount());
					if (priorityIndexArray.GetCount() <= minFail)
					{
						bestBrain.CopyFrom(m_brain);
						minFail = priorityIndexArray.GetCount();
						ndExpandTraceMessage("success rate: %f%%   ", (ndFloat32)(trainingDigits->GetCount() - minFail) * 100.0f / (ndFloat32)trainingDigits->GetCount());
						ndExpandTraceMessage("failed count: %d   ", minFail);
						ndExpandTraceMessage("steps: %d", i);
						ndExpandTraceMessage("\n");
					}
				}
			}
			m_brain.CopyFrom(bestBrain);
		}

		ndBrain& m_brain;
		ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_optimizers;
		ndReal m_learnRate;
		ndInt32 m_bashBufferSize;
	};
	
	if (trainingLabels && trainingDigits)
	{
		ndBrain brain;
		ndInt32 neuronsPerLayers = 64;
#if 0
		ndBrainLayerLinearActivated* const inputLayer = new ndBrainLayerLinearActivated(trainingDigits->GetColumns(), neuronsPerLayers, m_tanh);
		ndBrainLayerLinearActivated* const hiddenLayer0 = new ndBrainLayerLinearActivated(inputLayer->GetOuputSize(), neuronsPerLayers, m_tanh);
		ndBrainLayerLinearActivated* const hiddenLayer1 = new ndBrainLayerLinearActivated(hiddenLayer0->GetOuputSize(), neuronsPerLayers, m_tanh);
		ndBrainLayerLinearActivated* const ouputLayer = new ndBrainLayerLinearActivated(hiddenLayer1->GetOuputSize(), trainingLabels->GetColumns(), m_sigmoid);
	
		brain.BeginAddLayer();
		brain.AddLayer(inputLayer);
		brain.AddLayer(hiddenLayer0);
		brain.AddLayer(hiddenLayer1);
		brain.AddLayer(ouputLayer);
		brain.EndAddLayer();
#else
		ndBrainLayer* const inputLayer = new ndBrainLayerLineal(trainingDigits->GetColumns(), neuronsPerLayers);
		brain.AddLayer(inputLayer);
#endif
		brain.InitWeightsXavierMethod();
	
		SupervisedTrainer optimizer(&brain);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *trainingDigits);
		time = ndGetTimeInMicroseconds() - time;
	
		char path[256];
		ndGetWorkingFileName("mnistDatabase/mnist.nn", path);
		
		ndBrainSave::Save(&brain, path);
		ValidateData("training data", brain, *trainingLabels, *trainingDigits);
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

static void MnistTestSet()
{
	ndSharedPtr<ndBrainMatrix> testLabels (LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> testDigits (LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte"));

	if (testLabels && testDigits)
	{
		char path[256];
		ndGetWorkingFileName("mnistDatabase/mnist.nn", path);
		
		ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path));
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		ValidateData("test data", *(*brain), *testLabels, *testDigits);
		time = ndGetTimeInMicroseconds() - time;
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

void ndTestDeedBrian()
{
	ndSetRandSeed(12345);

	//FILE* outFile = fopen("xxx.csv", "wb");
	//for (ndInt32 i = 0; i < 40000; ++i)
	//{
	//	ndReal xxx = ndGaussianRandom(0.0f, 0.1f);
	//	fprintf(outFile, "%g\n", xxx);
	//}
	//fclose(outFile);

	ThreeLayersTwoInputsTwoOutputs();
	//MnistTrainingSet();
	//MnistTestSet();
	exit(0);
}
