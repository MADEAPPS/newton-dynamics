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
	ndDeepBrain brain;
	ndSetRandSeed(12345);

	ndDeepBrainLayer* const inputLayer = new ndDeepBrainLayer(2, 16, m_relu);
	//ndDeepBrainLayer* const hiddenLayer = new ndDeepBrainLayer(inputLayer->GetOuputSize(), 16, m_relu);
	ndDeepBrainLayer* const hiddenLayer = new ndDeepBrainLayer(inputLayer->GetOuputSize(), 6, m_relu);
	ndDeepBrainLayer* const ouputLayer = new ndDeepBrainLayer(hiddenLayer->GetOuputSize(), 2, m_sigmoid);

	brain.BeginAddLayer();
	brain.AddLayer(inputLayer);
	brain.AddLayer(hiddenLayer);
	brain.AddLayer(ouputLayer);
	brain.EndAddLayer();
	brain.InitGaussianWeights(0.0f, 0.25f);

	ndInt32 samples = 2000;
	ndDeepBrainMatrix inputBatch(samples, 2);
	ndDeepBrainMatrix groundTruth(samples, 2);
	for (ndInt32 i = 0; i < samples; i++)
	{
		inputBatch[i][0] = ndGaussianRandom(0.5f, 0.25f);
		inputBatch[i][1] = ndGaussianRandom(0.5f, 0.25f);
	
		groundTruth[i][0] = ((inputBatch[i][0] >= 0.5f) && (inputBatch[i][1] >= 0.5f)) ? 1.0f : 0.0f;
		groundTruth[i][1] = ((inputBatch[i][0] >= 0.5f) || (inputBatch[i][1] >= 0.5f)) ? 1.0f : 0.0f;
	}

	class Validator : public ndDeepBrainTrainer::ndValidation
	{
		public: 
		Validator(ndDeepBrainTrainer& trainer)
			:ndDeepBrainTrainer::ndValidation(trainer)
		{
		}

		ndReal Validate(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth)
		{
			ndReal error = ndDeepBrainTrainer::ndValidation::Validate(inputBatch, groundTruth);
			ndExpandTraceMessage("%f\n", error);
			return error;
		}
	};
	
	ndDeepBrainTrainer trainer(&brain, 1.0e-6f);

	Validator testError(trainer);

	trainer.SetMiniBatchSize(16);
	//trainer.Optimize(inputBatch, groundTruth, 0.0e-2f, 5000);
	trainer.Optimize(testError, inputBatch, groundTruth, 1.0e-2f, 2000);
	//trainer.Optimize(inputBatch, groundTruth, 0.0e-3f, 100000);

	brain.Save("xxx.nn");
	//ndDeepBrain brain1;
	//brain1.Load("xxx.nn");
	//ndAssert(brain1.Compare(brain));

	ndDeepBrainVector truth;
	ndDeepBrainVector input;
	ndDeepBrainVector output;
	
	truth.SetCount(2);
	input.SetCount(2);
	output.SetCount(2);
	
	ndInt32 failCount = 0;
	ndInt32 testCount = 200;
	ndDeepBrainInstance instance(&brain);
	for (ndInt32 i = 0; i < testCount; i++)
	{
		input[0] = ndGaussianRandom(0.5f, 0.25f);
		input[1] = ndGaussianRandom(0.5f, 0.25f);
		truth[0] = ((input[0] >= 0.5f) && (input[1] >= 0.5f)) ? 1.0f : 0.0f;
		truth[1] = ((input[0] >= 0.5f) || (input[1] >= 0.5f)) ? 1.0f : 0.0f;
	
		instance.MakePrediction(input, output);

		bool predicted = true;
		for (ndInt32 j = 0; j < output.GetCount(); j++)
		{
			bool trueBit = truth[j] >= 0.5f;
			bool predictBit = output[j] >= 0.5f;
			predicted = predicted & (predictBit == trueBit);
		}

		if (!predicted)
		{
			failCount++;
		}
	}

	ndExpandTraceMessage("%s\n", "boolean logic");
	ndExpandTraceMessage("num_right: %d  out of %d\n", testCount - failCount, testCount);
	ndExpandTraceMessage("num_wrong: %d  out of %d\n", failCount, testCount);
	ndExpandTraceMessage("success rate %f%%\n", (testCount - failCount) * 100.0f / testCount);
}

static ndDeepBrainMatrix* LoadMnistLabelData(const char* const filename)
{
	ndDeepBrainMatrix* labelsData = nullptr;

	char outPathName[1024];
	dGetWorkingFileName(filename, outPathName);
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

		labelsData = new ndDeepBrainMatrix(numberOfItems, 10);
		labelsData->Set(0.0f);
		for (ndUnsigned32 i = 0; i < numberOfItems; ++i)
		{
			ndUnsigned8 label;
			ret = fread(&label, 1, 1, fp);
			(*labelsData)[i][label] = 1.0f;
		}
		fclose(fp);
	}
	return labelsData;
}

static ndDeepBrainMatrix* LoadMnistSampleData(const char* const filename)
{
	ndDeepBrainMatrix* trainingDigits = nullptr;

	char outPathName[1024];
	dGetWorkingFileName(filename, outPathName);
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
		trainingDigits = new ndDeepBrainMatrix(numberOfItems, digitWith * digitHeight);
		trainingDigits->Set(0.0f);

		ndUnsigned8 data[32 * 32];
		for (ndUnsigned32 i = 0; i < numberOfItems; ++i)
		{
			ndDeepBrainVector& image = (*trainingDigits)[i];
			ret = fread(data, digitWith, digitHeight, fp);
			for (ndUnsigned32 j = 0; j < digitWith * digitHeight; j++)
			{
				image[j] = ndReal(data[j]) / 255.0f;
			}
		}
		fclose(fp);
	}
	return trainingDigits;
}

static void ValidateData(const char* const title, ndDeepBrain& brain, ndDeepBrainMatrix* const testLabels, ndDeepBrainMatrix* const testDigits)
{
	ndDeepBrainInstance instance(&brain);

	ndDeepBrainVector output;
	output.SetCount((*testLabels)[0].GetCount());

	ndInt32 failCount = 0;
	for (ndInt32 i = 0; i < testDigits->GetCount(); i++)
	{
		const ndDeepBrainVector& input = (*testDigits)[i];
		instance.MakePrediction(input, output);

		const ndDeepBrainVector& truth = (*testLabels)[i];

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
	ndExpandTraceMessage("success rate %f%%\n", (testDigits->GetCount() - failCount) * 100.0f / testDigits->GetCount());
}

static void MnistTrainingSet()
{
	ndDeepBrainMatrix* trainingLabels = LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte");
	ndDeepBrainMatrix* trainingDigits = LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte");

	if (trainingLabels && trainingDigits)
	{
		ndDeepBrain brain;
		ndSetRandSeed(142543);

		ndInt32 neuronsPerLayers = 64;
		ndDeepBrainLayer* const inputLayer = new ndDeepBrainLayer(trainingDigits->GetColumns(), neuronsPerLayers, m_tanh);
		ndDeepBrainLayer* const hiddenLayer0 = new ndDeepBrainLayer(inputLayer->GetOuputSize(), neuronsPerLayers, m_tanh);
		ndDeepBrainLayer* const hiddenLayer1 = new ndDeepBrainLayer(hiddenLayer0->GetOuputSize(), neuronsPerLayers, m_tanh);
		//ndDeepBrainLayer* const hiddenLayer2 = new ndDeepBrainLayer(hiddenLayer1->GetOuputSize(), neuronsPerLayers, m_tanh);
		ndDeepBrainLayer* const ouputLayer = new ndDeepBrainLayer(hiddenLayer1->GetOuputSize(), trainingLabels->GetColumns(), m_sigmoid);

		brain.BeginAddLayer();
		brain.AddLayer(inputLayer);
		brain.AddLayer(hiddenLayer0);
		brain.AddLayer(hiddenLayer1);
		//brain.AddLayer(hiddenLayer2);
		brain.AddLayer(ouputLayer);
		brain.EndAddLayer();
		brain.InitGaussianWeights(0.0f, 0.25f);

		//ndDeepBrainTrainer trainer(&brain, 1.0e-6f);
		ndDeepBrainParallelTrainer trainer(&brain, 1.0e-6f, 4);
		//ndDeepBrainTrainerParallelSDG_Experiment trainer(&brain, 1.0e-6f, 4);

		ndUnsigned64 time = ndGetTimeInMicroseconds();
		trainer.SetMiniBatchSize(2000);
		trainer.Optimize(*trainingDigits, *trainingLabels, 1.0e-2f, 5000);

		trainer.SetMiniBatchSize(20000);
		trainer.Optimize(*trainingDigits, *trainingLabels, 1.0e-2f, 2000);
		time = ndGetTimeInMicroseconds() - time;

		char path[256];
		dGetWorkingFileName("mnistDatabase/mnist.nn", path);
		brain.Save(path);
		ValidateData("training data", brain, trainingLabels, trainingDigits);
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}

	if (trainingLabels)
	{
		delete trainingLabels;
	}

	if (trainingDigits)
	{
		delete trainingDigits;
	}
}

static void MnistTestSet()
{
	ndDeepBrainMatrix* testLabels = LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte");
	ndDeepBrainMatrix* testDigits = LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte");

	if (testLabels && testDigits)
	{
		char path[256];
		ndDeepBrain brain;
		dGetWorkingFileName("mnistDatabase/mnist.nn", path);
		brain.Load(path);
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
	ThreeLayersTwoInputsTwoOutputs();
	//MnistTrainingSet();
	//MnistTestSet();
}

