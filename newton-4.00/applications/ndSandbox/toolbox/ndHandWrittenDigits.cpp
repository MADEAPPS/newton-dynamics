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

//#define D_USE_CONVOLUTIONAL_LAYERS

static void ThreeLayersTwoInputsTwoOutputs()
{
	ndBrain brain;
	ndInt32 hiddenNeurons = 16;

	ndFixSizeArray<ndBrainLayer*, 16> layers;

	layers.PushBack(new ndBrainLayerLinear(2, hiddenNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(hiddenNeurons));

	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hiddenNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(hiddenNeurons));

	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hiddenNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(hiddenNeurons));

	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 2));
	layers.PushBack(new ndBrainLayerSigmoidActivation(2));
	
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		brain.AddLayer(layers[i]);
	}
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
	ndArray<ndBrainTrainer*> trainers;

	ndBrainThreadPool threads;
	threads.SetThreadCount(4);
	for (ndInt32 i = 0; i < bashSize; ++i)
	{
		trainers.PushBack(new ndBrainTrainer(&brain));
	}

	ndBrainOptimizerAdam optimizer;
	//ndBrainOptimizerSgd optimizer;

	ndInt32 randomeSelection[bashSize];
	auto UpdateTrainer = ndMakeObject::ndFunction([&trainers, &randomeSelection, &inputBatch, &groundTruth, bashSize](ndInt32 threadIndex, ndInt32 threadCount)
	{
		const ndStartEnd startEnd(bashSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBrainTrainer& trainer = *trainers[i];
			ndBrainLossLeastSquaredError loss(trainer.GetBrain()->GetOutputSize());

			ndInt32 index = randomeSelection[i];
			const ndBrainVector& input = inputBatch[index];
			loss.SetTruth(groundTruth[index]);
			trainer.BackPropagate(input, loss);
		}
	});

	for (ndInt32 i = 0; i < 5000; ++i)
	{
		for (ndInt32 j = 0; j < bashSize; ++j)
		{
			randomeSelection[j] = ndInt32 (ndRandInt() % samples);
		}
		threads.ParallelExecute(UpdateTrainer);
		optimizer.Update(&threads, trainers, ndBrainFloat(1.0e-3f));
	}
	
	ndBrainVector truth;
	ndBrainVector input;
	ndBrainVector output;
	ndBrainVector workBuffer;
	
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
	
		brain.MakePrediction(input, output, workBuffer);
	
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

	for (ndInt32 i = 0; i < trainers.GetCount(); ++i)
	{
		delete trainers[i];
	}

	ndExpandTraceMessage("%s\n", "boolean logic");
	ndExpandTraceMessage("num_right: %d  out of %d\n", testCount - failCount, testCount);
	ndExpandTraceMessage("num_wrong: %d  out of %d\n", failCount, testCount);
	ndExpandTraceMessage("success rate %f%%\n", (ndFloat32)(testCount - failCount) * 100.0f / (ndFloat32)testCount);
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
			image.GaussianNormalize();
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
	ndBrainVector workingBuffer;
	for (ndInt32 i = 0; i < testDigits->GetCount(); i++)
	{
		const ndBrainVector& input = (*testDigits)[i];
		brain.MakePrediction(input, output, workingBuffer);

		const ndBrainVector& truth = (*testLabels)[i];

		ndInt32 index = -1;
		ndBrainFloat maxProbability = -1.0f;
		for (ndInt32 j = 0; j < output.GetCount(); j++)
		{
			if (output[j] > maxProbability)
			{
				index = j;
				maxProbability = output[j];
			}
		}

		ndAssert(index >= 0);
		//if (truth[index] < 0.5f)
		if (truth[index] == ndReal(0.0f))
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
	#define BASH_BUFFER_SIZE	64

	ndSharedPtr<ndBrainMatrix> trainingLabels (LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> trainingDigits (LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte"));

	ndSharedPtr<ndBrainMatrix> testLabels(LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> testDigits(LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte"));
	
	class SupervisedTrainer : public ndBrainThreadPool
	{
		public:
		SupervisedTrainer(ndBrain* const brain)
			:ndBrainThreadPool()
			,m_brain(*brain)
			,m_learnRate(ndReal(1.0e-4f))
			,m_bashBufferSize(BASH_BUFFER_SIZE)
		{
			ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), ndMin(m_bashBufferSize, 16));
	//threadCount = 1;
			SetThreadCount(threadCount);
			for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
			{
				ndBrainTrainer* const trainer = new ndBrainTrainer(&m_brain);
				m_trainers.PushBack(trainer);
			}
			m_prioritySamples.SetCount(m_bashBufferSize);
		}

		~SupervisedTrainer()
		{
			for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
			{
				delete m_trainers[i];
			}
		}

		void Optimize(ndBrainMatrix* const trainingLabels, ndBrainMatrix* const trainingDigits,
					  ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			ndUnsigned32 miniBashArray[64];
			ndUnsigned32 failCount[D_MAX_THREADS_COUNT];

			auto BackPropagateBash = ndMakeObject::ndFunction([this, trainingDigits, trainingLabels, &miniBashArray, &failCount](ndInt32 threadIndex, ndInt32 threadCount)
			{
				class CategoricalLoss : public ndBrainLossCategoricalCrossEntropy
				{
					public:
					CategoricalLoss(ndInt32 size, ndUnsigned32* const failCount, ndFixSizeArray<ndUnsigned32, 16>& priority, ndUnsigned32 entry)
						:ndBrainLossCategoricalCrossEntropy(size)
						,m_entry(entry)
						,m_failCount(failCount)
						,m_priority(priority)
					{
					}

					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						ndInt32 index = -1;
						ndBrainFloat maxProbability = ndBrainFloat(-1.0f);
						for (ndInt32 j = 0; j < output.GetCount(); j++)
						{
							if (output[j] > maxProbability)
							{
								index = j;
								maxProbability = output[j];
							}
						}

						ndAssert(index >= 0);
						if (m_truth[index] == ndReal(0.0f))
						{
							(*m_failCount)++;
							if (m_priority.GetCount() < m_priority.GetCapacity())
							{
								m_priority.PushBack(m_entry);
							}
						}

						ndBrainLossCategoricalCrossEntropy::GetLoss(output, loss);
					}

					ndUnsigned32 m_entry;
					ndUnsigned32* m_failCount;
					ndFixSizeArray<ndUnsigned32, 16>& m_priority;
				};

				const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndBrainTrainer& trainer = *m_trainers[i];
					ndUnsigned32 index = miniBashArray[i];
					CategoricalLoss loss(m_brain.GetOutputSize(), &failCount[threadIndex], m_prioritySamples[i], index);
				
					loss.SetTruth((*trainingLabels)[ndInt32(index)]);
					trainer.BackPropagate((*trainingDigits)[ndInt32(index)], loss);
				}
			});

			ndBrain bestBrain(m_brain);
			ndBrainOptimizerAdam optimizer;

			ndInt32 minTestFail = testDigits->GetCount();
			ndInt32 minTrainingFail = trainingDigits->GetCount();
			ndInt32 batches = trainingDigits->GetCount() / m_bashBufferSize;

			// so far best training result on the mnist data set
			//optimizer.SetRegularizer(ndBrainFloat(0.0e-5f)); // test data score fully(96.76%) conv(96.449997%)
			//optimizer.SetRegularizer(ndBrainFloat(1.0e-5f)); // test data score fully(98.02%)  conv(97.529999%)
			//optimizer.SetRegularizer(ndBrainFloat(2.0e-5f)); // test data score fully(98.08%)  conv(97.760002%)
			//optimizer.SetRegularizer(ndBrainFloat(3.0e-5f)); // test data score fully(98.18%)  conv(97.339996%)
			optimizer.SetRegularizer(ndBrainFloat(4.0e-5f)); // test data score fully(97.90%)  conv(97.339996%)

			//batches = 1;
			ndArray<ndUnsigned32> shuffleBuffer;
			for (ndInt32 i = 0; i < trainingDigits->GetCount(); ++i)
			{
				shuffleBuffer.PushBack(ndUnsigned32(i));
			}
			ndFixSizeArray<ndUnsigned32, 1024 * 2> priorityList;
			for (ndInt32 i = 0; i < priorityList.GetCapacity(); ++i)
			{
				priorityList.PushBack(ndRandInt() % trainingDigits->GetCount());
			}

			for (ndInt32 epoch = 0; epoch < 1000; ++epoch)
			{
				ndInt32 start = 0;
				ndMemSet(failCount, ndUnsigned32(0), D_MAX_THREADS_COUNT);

				for (ndInt32 bash = 0; bash < batches; ++bash)
				{
					ndMemCpy(miniBashArray, &shuffleBuffer[start], m_bashBufferSize);

					const ndInt32 priorityCount = ndMin(4, priorityList.GetCount());
					for (ndInt32 i = 0; i < priorityCount; ++i)
					{
						miniBashArray[i] = priorityList[ndInt32 (ndRandInt() % priorityList.GetCount())];
					}

					ndBrainThreadPool::ParallelExecute(BackPropagateBash);
					optimizer.Update(this, m_trainers, m_learnRate);

					start += m_bashBufferSize;
				}

				ndInt32 fails = 0;
				for (ndInt32 i = 0; i < GetThreadCount(); ++i)
				{
					fails += failCount[i];
				}

				if (fails <= minTrainingFail)
				{
					ndInt32 actualTraining = fails;
					bool traningTest = fails < minTrainingFail;
					minTrainingFail = ndMax(fails, ndInt32(500));

					auto CrossValidateTest = ndMakeObject::ndFunction([this, testDigits, testLabels, &failCount](ndInt32 threadIndex, ndInt32 threadCount)
					{
						ndBrainFloat outputBuffer[32];
						ndBrainMemVector output(outputBuffer, m_brain.GetOutputSize());

						failCount[threadIndex] = 0;
						const ndStartEnd startEnd(testDigits->GetCount(), threadIndex, threadCount);
						for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
						{
							const ndBrainVector& truth = (*testLabels)[i];
							const ndBrainVector& input = (*testDigits)[i];
							m_brain.MakePrediction(input, output, m_trainers[threadIndex]->GetWorkingBuffer());

							ndInt32 index = -1;
							ndBrainFloat maxProbability = ndBrainFloat(-1.0f);
							for (ndInt32 j = 0; j < output.GetCount(); j++)
							{
								if (output[j] > maxProbability)
								{
									index = j;
									maxProbability = output[j];
								}
							}
							if (truth[index] == ndReal(0.0f))
							{
								failCount[threadIndex]++;
							}
						}
					});

					ndBrainThreadPool::ParallelExecute(CrossValidateTest);

					fails = 0;
					for (ndInt32 j = 0; j < GetThreadCount(); ++j)
					{
						fails += failCount[j];
					}

					if (traningTest && (minTrainingFail > 500))
					{
						minTestFail = fails;
						bestBrain.CopyFrom(m_brain);
						ndInt32 size = batches * m_bashBufferSize;
						ndExpandTraceMessage("success rate: %f%%   ", (ndFloat32)(size - actualTraining) * 100.0f / (ndFloat32)size);
						ndExpandTraceMessage("failed count: %d   ", actualTraining);
						ndExpandTraceMessage("epoch: %d", epoch);
						ndExpandTraceMessage("\n");
					}
					else if (fails <= minTestFail)
					{
						minTestFail = fails;
						bestBrain.CopyFrom(m_brain);
						ndInt32 size = batches * m_bashBufferSize;
						ndExpandTraceMessage("success rate: %f%%   ", (ndFloat32)(size - actualTraining) * 100.0f / (ndFloat32)size);
						ndExpandTraceMessage("failed count: %d   ", actualTraining);
						ndExpandTraceMessage("epoch: %d", epoch);
						ndExpandTraceMessage(" %d\n", minTestFail);
					}
				}

				priorityList.SetCount(0);
				for (ndInt32 i = 0; i < m_prioritySamples.GetCount(); ++i)
				{
					ndFixSizeArray<ndUnsigned32, 16>& priority = m_prioritySamples[i];
					for (ndInt32 j = 0; j < priority.GetCount(); ++j)
					{
						priorityList.PushBack(priority[j]);
					}
					priority.SetCount(0);
				}
				shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
			}
			m_brain.CopyFrom(bestBrain);
		}

		ndBrain& m_brain;
		ndArray<ndBrainTrainer*> m_trainers;
		ndFixSizeArray<ndFixSizeArray<ndUnsigned32, 16>, BASH_BUFFER_SIZE> m_prioritySamples;
		ndReal m_learnRate;
		ndInt32 m_bashBufferSize;
	};
	
	if (trainingLabels && trainingDigits)
	{
		ndBrain brain;
		ndFixSizeArray<ndBrainLayer*, 32> layers;
		
#ifdef D_USE_CONVOLUTIONAL_LAYERS
		ndInt32 height = 28;
		ndInt32 width = trainingDigits->GetColumns() / height;
		ndAssert((height * width) == trainingDigits->GetColumns());

		const ndBrainLayerConvolutional* conv;
		const ndBrainLayerConvolutionalMaxPooling* pooling;

		#if 1
			#define DIGIT_ACTIVATION_TYPE ndBrainLayerReluActivation
		#else
			#define DIGIT_ACTIVATION_TYPE ndBrainLayerTanhActivation
		#endif

		layers.PushBack(new ndBrainLayerConvolutional(width, height, 1, 3, 16));
		conv = (ndBrainLayerConvolutional*)(layers[layers.GetCount() - 1]);
		layers.PushBack(new DIGIT_ACTIVATION_TYPE(conv->GetOutputSize()));
		layers.PushBack(new ndBrainLayerConvolutionalMaxPooling(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
		pooling = (ndBrainLayerConvolutionalMaxPooling*)(layers[layers.GetCount() - 1]);

		layers.PushBack(new ndBrainLayerConvolutional(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 32));
		conv = (ndBrainLayerConvolutional*)(layers[layers.GetCount() - 1]);
		layers.PushBack(new DIGIT_ACTIVATION_TYPE(conv->GetOutputSize()));
		layers.PushBack(new ndBrainLayerConvolutionalMaxPooling(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
		pooling = (ndBrainLayerConvolutionalMaxPooling*)(layers[layers.GetCount() - 1]);

		layers.PushBack(new ndBrainLayerConvolutional(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 32));
		conv = (ndBrainLayerConvolutional*)(layers[layers.GetCount() - 1]);
		layers.PushBack(new DIGIT_ACTIVATION_TYPE(conv->GetOutputSize()));

		ndInt32 neuronsPerLayers = 64;
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
		layers.PushBack(new DIGIT_ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
		layers.PushBack(new ndBrainLayerCategoricalSoftmaxActivation(layers[layers.GetCount() - 1]->GetOutputSize()));

#else
		
		ndInt32 neuronsPerLayers = 64;
		layers.PushBack(new ndBrainLayerLinear(trainingDigits->GetColumns(), neuronsPerLayers));
		layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
		layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
		
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
		layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
		
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
		layers.PushBack(new ndBrainLayerCategoricalSoftmaxActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
#endif

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain.AddLayer(layers[i]);
		}
		brain.InitWeightsXavierMethod();
	
		SupervisedTrainer optimizer(&brain);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *trainingDigits, *testLabels, *testDigits);
		time = ndGetTimeInMicroseconds() - time;
	
		char path[256];
		#ifdef D_USE_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif
		
		ndBrainSave::Save(&brain, path);
		ValidateData("training data", brain, *trainingLabels, *trainingDigits);
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
		#ifdef D_USE_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif

		ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path));
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		ValidateData("test data", *(*brain), *testLabels, *testDigits);
		time = ndGetTimeInMicroseconds() - time;
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

void ndHandWrittenDigits()
{
	ndSetRandSeed(12345);

	//int xxxxxxx = ndRandInt();
	//ndFloat32 xxxxxxx0 = ndRand();
	//ndBrainVector xxx;
	//for (ndInt32 i = 0; i < 100000; ++i)
	//{
	//	//xxx.PushBack(ndGaussianRandom(0.0f, 0.1f));
	//	xxx.PushBack(1.0f);
	//}
	//xxx.GaussianNormalize();

	//ThreeLayersTwoInputsTwoOutputs();
	MnistTrainingSet();
	MnistTestSet();
}
