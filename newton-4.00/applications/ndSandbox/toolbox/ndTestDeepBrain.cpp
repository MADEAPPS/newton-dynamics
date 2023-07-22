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
	ndInt32 neurons = 32;
	ndBrainLayer* const inputLayer = new ndBrainLayer(2, neurons, m_relu);
	ndBrainLayer* const hiddenLayer0 = new ndBrainLayer(inputLayer->GetOuputSize(), neurons, m_relu);
	ndBrainLayer* const hiddenLayer1 = new ndBrainLayer(hiddenLayer0->GetOuputSize(), neurons, m_relu);
	ndBrainLayer* const ouputLayer = new ndBrainLayer(hiddenLayer1->GetOuputSize(), 2, m_sigmoid);
	
	brain.BeginAddLayer();
	brain.AddLayer(inputLayer);
	brain.AddLayer(hiddenLayer0);
	brain.AddLayer(hiddenLayer1);
	brain.AddLayer(ouputLayer);
	brain.EndAddLayer(ndReal(0.125f));
	
	ndInt32 samples = 2000;
	ndBrainMatrix inputBatch(samples, 2);
	ndBrainMatrix groundTruth(samples, 2);
	ndArray<ndInt32> randomeSelection;
	for (ndInt32 i = 0; i < samples; i++)
	{
		randomeSelection.PushBack(i);
		inputBatch[i][0] = ndClamp (ndReal(ndGaussianRandom(0.5f, 0.25f)), ndReal(0.0f), ndReal(1.0f));
		inputBatch[i][1] = ndClamp (ndReal(ndGaussianRandom(0.5f, 0.25f)), ndReal(0.0f), ndReal(1.0f));
	
		groundTruth[i][0] = ((inputBatch[i][0] >= ndReal(0.5f)) && (inputBatch[i][1] >= ndReal(0.5f))) ? ndReal(1.0f) : ndReal(0.0f);
		groundTruth[i][1] = ((inputBatch[i][0] >= ndReal(0.5f)) || (inputBatch[i][1] >= ndReal(0.5f))) ? ndReal(1.0f) : ndReal(0.0f);
	}
	
	const ndInt32 bashSize = 64;
	//ndBrainTrainer trainer(&brain);
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> trainers;

	//ndBrainLeastSquareErrorLoss loss(trainer.GetBrain().GetOutputSize());
	//for (ndInt32 i = 0; i < 20000; ++i)
	//{
	//	trainer.ClearGradientsAcc();
	//	randomeSelection.RandomShuffle(randomeSelection.GetCount());
	//	for (ndInt32 j = 0; j < bashSize; ++j)
	//	{
	//		ndInt32 index = randomeSelection[j];
	//		const ndBrainVector& input = inputBatch[index];
	//		//const ndBrainVector& truth = groundTruth[index];
	//		//trainer.BackPropagate(input, truth);
	//		loss.SetTruth(groundTruth[index]);
	//		trainer.BackPropagate(input, loss);
	//	}
	//	trainer.UpdateWeights(ndReal(1.0e-2f), bashSize);
	//}

	ndBrainThreadPool threads;
	threads.SetThreadCount(4);
	for (ndInt32 i = 0; i < threads.GetThreadCount(); ++i)
	{
		trainers.PushBack(new ndBrainTrainer(&brain));
	}

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
			loss.SetTruth(groundTruth[index]);
			trainer.BackPropagate(input, loss);
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
		randomeSelection.RandomShuffle(randomeSelection.GetCount());
		threads.ParallelExecute(UpdateTrainer);
		threads.ParallelExecute(AccumulateWeight);
		trainers[0]->UpdateWeights(ndReal(1.0e-2f), bashSize);
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
	ndAssert(0);
	//ndBrainMatrix* trainingLabels = LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte");
	//ndBrainMatrix* trainingDigits = LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte");
	//
	//if (trainingLabels && trainingDigits)
	//{
	//	ndBrain brain;
	//
	//	ndInt32 neuronsPerLayers = 64;
	//	ndBrainLayer* const inputLayer = new ndBrainLayer(trainingDigits->GetColumns(), neuronsPerLayers, m_tanh);
	//	ndBrainLayer* const hiddenLayer0 = new ndBrainLayer(inputLayer->GetOuputSize(), neuronsPerLayers, m_tanh);
	//	ndBrainLayer* const hiddenLayer1 = new ndBrainLayer(hiddenLayer0->GetOuputSize(), neuronsPerLayers, m_tanh);
	//	//ndBrainLayer* const hiddenLayer2 = new ndBrainLayer(hiddenLayer1->GetOuputSize(), neuronsPerLayers, m_tanh);
	//	ndBrainLayer* const ouputLayer = new ndBrainLayer(hiddenLayer1->GetOuputSize(), trainingLabels->GetColumns(), m_sigmoid);
	//
	//	brain.BeginAddLayer();
	//	brain.AddLayer(inputLayer);
	//	brain.AddLayer(hiddenLayer0);
	//	brain.AddLayer(hiddenLayer1);
	//	//brain.AddLayer(hiddenLayer2);
	//	brain.AddLayer(ouputLayer);
	//	brain.EndAddLayer(ndReal(0.125f));
	//
	//	ndBrainTrainer trainer(&brain);
	//	//ndBrainParallelTrainer trainer(&brain, 4);
	//	//ndDeepBrainTrainerParallelSDG_Experiment trainer(&brain, 4);
	//
	//	trainer.SetMiniBatchSize(16);
	//	ndTestValidator validator(trainer);
	//
	//	ndUnsigned64 time = ndGetTimeInMicroseconds();
	//	//trainer.Optimize(validator, *trainingDigits, *trainingLabels, 5.0e-3f, 20);
	//	trainer.Optimize(validator, *trainingDigits, *trainingLabels, 2000);
	//	time = ndGetTimeInMicroseconds() - time;
	//
	//	char path[256];
	//	ndGetWorkingFileName("mnistDatabase/mnist.nn", path);
	//	
	//	ndAssert(0);
	//	//brain.Save(path);
	//	ValidateData("training data", brain, trainingLabels, trainingDigits);
	//	ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	//}
	//
	//if (trainingLabels)
	//{
	//	delete trainingLabels;
	//}
	//
	//if (trainingDigits)
	//{
	//	delete trainingDigits;
	//}
}

static void MnistTestSet()
{
	ndBrainMatrix* testLabels = LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte");
	ndBrainMatrix* testDigits = LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte");

	if (testLabels && testDigits)
	{
		char path[256];
		ndBrain brain;
		ndGetWorkingFileName("mnistDatabase/mnist.nn", path);
		
		ndAssert(0);
		//brain.Load(path);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		ValidateData("test data", brain, testLabels, testDigits);
		time = ndGetTimeInMicroseconds() - time;
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}

	if (testLabels)
	{
		delete testLabels;
	}

	if (testDigits)
	{
		delete testDigits;
	}
}

void ndTestDeedBrian()
{
	ndSetRandSeed(12345);
	//ThreeLayersTwoInputsTwoOutputs();
	//MnistTrainingSet();
	//MnistTestSet();
}
