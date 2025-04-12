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

		size_t ret = 0;
		ndUnsigned32 magicNumber;
		ndUnsigned32 numberOfItems;
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
	brain.DisableDropOut();
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

static void ValidateDataGpu(const char* const title, ndBrain& brain, ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
{
	//ndInt32 batchSize = 100;
	const ndInt32 batchSize = ndInt32(testDigits->GetCount());

	ndBrainGpuContext gpuContext;
	ndBrainGpuInference inference(&gpuContext, &brain, *testDigits, batchSize);

	ndUnsigned64 gpuTime = ndGetTimeInMicroseconds();
	gpuContext.SubmitQueue(inference.GetDisplayList());
	gpuContext.Sync();
	gpuTime = ndGetTimeInMicroseconds() - gpuTime;
	ndExpandTraceMessage("gpuTime %f (sec) batch size(%d)\n", ndFloat64(gpuTime) / 1000000.0f, batchSize);

	ndBrainVector workBuffer;
	ndBrainVector outputBuffer;
	inference.GetResults(outputBuffer);

	ndInt32 failCount = 0;
	const ndInt32 outputSize = ndInt32((*testLabels)[0].GetCount());

	ndBrainVector workingBuffer;
	brain.DisableDropOut();

	ndBrainVector output;
	output.SetCount(outputSize * batchSize);
	ndUnsigned64 cpuTime = ndGetTimeInMicroseconds();
	for (ndInt32 i = 0; i < batchSize; i++)
	{
		const ndBrainVector& input = (*testDigits)[i];
		ndBrainMemVector outputCpu (&output[i * outputSize], outputSize);
		brain.MakePrediction(input, outputCpu, workingBuffer);
		//#ifdef _DEBUG
		//const ndArray<ndInt32>& offsets = inference.GetWorkBufferOffsets();		
		//ndBrainMemVector workBufferBatch (&workBuffer[i * offsets[offsets.GetCount() - 1]], offsets[offsets.GetCount() - 1]);
		//brain.MakePrediction_____(input, outputCpu, workingBuffer, workBufferBatch, offsets);
		//#endif
	}
	cpuTime = ndGetTimeInMicroseconds() - cpuTime;
	ndExpandTraceMessage("cpuTime %f (sec)\n", ndFloat64(cpuTime) / 1000000.0f);
	
	for (ndInt32 i = 0; i < batchSize; i++)
	{
		const ndBrainMemVector outputCpu(&output[i * outputSize], outputSize);
		const ndBrainMemVector outputGpu(&outputBuffer[i * outputSize], outputSize);
		const ndBrainVector& truth = (*testLabels)[i];
	
		ndInt32 indexCpu = -1;
		ndInt32 indexGpu = -1;
		ndBrainFloat maxProbability = -1.0f;
		ndBrainFloat maxProbabilityGpu = -1.0f;
		for (ndInt32 j = 0; j < outputSize; j++)
		{
			if (outputCpu[j] > maxProbability)
			{
				indexCpu = j;
				maxProbability = outputCpu[j];
			}
			if (outputGpu[j] > maxProbabilityGpu)
			{
				indexGpu = j;
				maxProbabilityGpu = outputGpu[j];
			}
		}
		ndAssert(indexGpu == indexCpu);
	
		ndAssert(indexCpu >= 0);
		if (truth[indexCpu] == ndReal(0.0f))
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
	//#define USE_CONVOLUTIONAL_LAYERS

	#define BATCH_BUFFER_SIZE				64
	#define CONVOLUTIONAL_FEATURE_MAPS		32
	#define MIN_TRAIN_SCORE					0.9999f

	#if 1
		#define CONVOLUTIONAL_LAYER	ndBrainLayerConvolutional_2d
	#else
		#define CONVOLUTIONAL_LAYER	ndBrainLayerConvolutionalWithDropOut_2d
	#endif

	#if 1
		#define LINEAR_LAYERS_NEURONS	64
		#define LINEAR_LAYER			ndBrainLayerLinear
	#else
		#define LINEAR_LAYERS_NEURONS	128
		#define LINEAR_LAYER			ndBrainLayerLinearWithDropOut
	#endif

	#if 0
		#define ACTIVATION_TYPE ndBrainLayerActivationTanh
	#else
		#define ACTIVATION_TYPE ndBrainLayerActivationElu
		//#define ACTIVATION_TYPE ndBrainLayerActivationRelu		
		//#define ACTIVATION_TYPE ndBrainLayerActivationSigmoidLinear
	#endif

	//#if 1
	//	//#define DIGIT_FILTER_LAYER_TYPE ndBrainLayerConvolutional_2d
	//	#define DIGIT_FILTER_LAYER_TYPE ndBrainLayerConvolutionalWithDropOut_2d
	//#else
	//	#define DIGIT_FILTER_LAYER_TYPE ndBrainLayerCrossCorrelation_2d			
	//#endif

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
			,m_prioritySamples()
			,m_learnRate(ndReal(5.0e-4f))
			,m_bashBufferSize(BATCH_BUFFER_SIZE)
		{
			ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);

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

		ndFloat32 LogScore(ndInt32 epoch, ndInt32 size) const
		{
			ndFloat32 score = (ndFloat32)(size - m_minTrainingFail) / (ndFloat32)size;
			ndExpandTraceMessage("  epoch: %d", epoch);
			ndExpandTraceMessage("  success rate:%f%%", score * 100.0f);
			ndExpandTraceMessage("  training fail count:%d", m_minTrainingFail);
			ndExpandTraceMessage("  test fail count:%d\n", m_minTestFail);
			return score;
		}

		ndInt32 ScoreModeZero(ndBrain& bestBrain, ndInt32 epoch, ndUnsigned32* const failCount, ndBrainMatrix* const trainingDigits, ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			ndInt32 trainFail = 0;
			for (ndInt32 i = 0; i < GetThreadCount(); ++i)
			{
				trainFail += failCount[i];
			}

			bool test = trainFail <= m_minTrainingFail;
			if (test)
			{
				ndAtomic<ndInt32> iterator(0);
				auto CrossValidateTest = ndMakeObject::ndFunction([this, &iterator, testDigits, testLabels, &failCount](ndInt32 threadIndex, ndInt32)
				{
					ndBrainFloat outputBuffer[32];
					ndBrainMemVector output(outputBuffer, m_brain.GetOutputSize());

					failCount[threadIndex] = 0;
					for (ndInt32 i = iterator++; i < testLabels->GetCount(); i = iterator++)
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

				iterator = 0;
				ndBrainThreadPool::ParallelExecute(CrossValidateTest);

				ndInt32 testFail = 0;
				for (ndInt32 j = 0; j < GetThreadCount(); ++j)
				{
					testFail += failCount[j];
				}

				m_minTestFail = testFail;
				m_minTrainingFail = trainFail;
				bestBrain.CopyFrom(m_brain);
				ndFloat32 score = LogScore(epoch, ndInt32(trainingDigits->GetCount()));
				return (score <= MIN_TRAIN_SCORE) ? 0 : 1;
			}
			return 0;
		}

		ndInt32 ScoreModeOne(ndBrain& bestBrain, ndInt32 epoch, ndUnsigned32* const failCount, ndBrainMatrix* const trainingDigits, ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			ndInt32 trainFail = 0;
			for (ndInt32 i = 0; i < GetThreadCount(); ++i)
			{
				trainFail += failCount[i];
			}

			ndFloat32 score = (ndFloat32)(trainingDigits->GetCount() - trainFail) / (ndFloat32)trainingDigits->GetCount();
			if (score > MIN_TRAIN_SCORE)
			{
				ndAtomic<ndInt32> iterator(0);
				auto CrossValidateTest = ndMakeObject::ndFunction([this, &iterator, testDigits, testLabels, &failCount](ndInt32 threadIndex, ndInt32)
				{
					ndBrainFloat outputBuffer[32];
					ndBrainMemVector output(outputBuffer, m_brain.GetOutputSize());

					failCount[threadIndex] = 0;
					for (ndInt32 i = iterator++; i < testLabels->GetCount(); i = iterator++)
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

				iterator = 0;
				ndBrainThreadPool::ParallelExecute(CrossValidateTest);

				ndInt32 testFail = 0;
				for (ndInt32 j = 0; j < GetThreadCount(); ++j)
				{
					testFail += failCount[j];
				}

				bool test = (testFail < m_minTestFail) || ((testFail == m_minTestFail) && (trainFail < m_minTrainingFail));
				if (test)
				{
					m_minTestFail = testFail;
					m_minTrainingFail = trainFail;
					bestBrain.CopyFrom(m_brain);
					LogScore(epoch, ndInt32(trainingDigits->GetCount()));
				}
			}
			return 0;
		}

		void Optimize(ndBrainMatrix* const trainingLabels, ndBrainMatrix* const trainingDigits,
					  ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			ndUnsigned32 failCount[D_MAX_THREADS_COUNT];
			ndUnsigned32 miniBashArray[BATCH_BUFFER_SIZE];

			ndAtomic<ndInt32> iterator(0);
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, trainingDigits, trainingLabels, &miniBashArray, &failCount](ndInt32 threadIndex, ndInt32)
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

				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
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

			m_minTestFail = ndInt32(testDigits->GetCount());
			m_minTrainingFail = ndInt32(trainingDigits->GetCount());
			ndInt32 batches = m_minTrainingFail / m_bashBufferSize;

			// so far best training result on the mnist data set
			optimizer.SetRegularizer(ndBrainFloat(0.0f));		//         training(100.0%) test(99.35%) 
			//optimizer.SetRegularizer(ndBrainFloat(0.0f));		// dropout training(99.998%) test(99.4%) 
			//optimizer.SetRegularizer(ndBrainFloat(1.0e-5f));	// 
			//optimizer.SetRegularizer(ndBrainFloat(2.0e-5f));	// 
			//optimizer.SetRegularizer(ndBrainFloat(3.0e-5f));	// 
			//optimizer.SetRegularizer(ndBrainFloat(4.0e-5f));	// 

			ndArray<ndUnsigned32> shuffleBuffer;
			for (ndInt32 i = 0; i < trainingDigits->GetCount(); ++i)
			{
				shuffleBuffer.PushBack(ndUnsigned32(i));
			}

			#ifdef USE_CONVOLUTIONAL_LAYERS
				const ndInt32 numberOfEpocks = 50;
			#else
				const ndInt32 numberOfEpocks = 400;
			#endif
			ndInt32 scoreMode = 0;
			for (ndInt32 epoch = 0; epoch < numberOfEpocks; ++epoch)
			{
				ndInt32 start = 0;
				ndMemSet(failCount, ndUnsigned32(0), D_MAX_THREADS_COUNT);

				m_brain.EnableDropOut();
				m_brain.UpdateDropOut();
				shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
				for (ndInt32 bash = 0; bash < batches; ++bash)
				{
					iterator = 0;
					ndMemCpy(miniBashArray, &shuffleBuffer[start], m_bashBufferSize);
					ndBrainThreadPool::ParallelExecute(BackPropagateBash);
					optimizer.Update(this, m_trainers, m_learnRate);
					start += m_bashBufferSize;
				}
				m_brain.DisableDropOut();

				if (scoreMode == 0)
				{
					scoreMode = ScoreModeZero(bestBrain, epoch, failCount, trainingDigits, testLabels, testDigits);
				}
				else
				{
					ScoreModeOne(bestBrain, epoch, failCount, trainingDigits, testLabels, testDigits);
				}
			}
			m_brain.CopyFrom(bestBrain);
		}

		ndBrain& m_brain;
		ndArray<ndBrainTrainer*> m_trainers;
		ndFixSizeArray<ndFixSizeArray<ndUnsigned32, 16>, BATCH_BUFFER_SIZE> m_prioritySamples;
		ndReal m_learnRate;
		ndInt32 m_bashBufferSize;
		ndInt32 m_minTestFail;
		ndInt32 m_minTrainingFail;

	};
	
	if (trainingLabels && trainingDigits)
	{
		ndBrain brain;
		ndFixSizeArray<ndBrainLayer*, 32> layers;

		#ifdef USE_CONVOLUTIONAL_LAYERS
			ndInt32 height = 28;
			ndInt32 width = trainingDigits->GetColumns() / height;
			ndAssert((height * width) == trainingDigits->GetColumns());

			const CONVOLUTIONAL_LAYER* conv;
			const ndBrainLayerImagePolling_2x2* pooling;

			layers.PushBack(new CONVOLUTIONAL_LAYER(width, height, 1, 3, CONVOLUTIONAL_FEATURE_MAPS));
			conv = (CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			layers.PushBack(new CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, CONVOLUTIONAL_FEATURE_MAPS));
			conv = (CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			layers.PushBack(new CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, CONVOLUTIONAL_FEATURE_MAPS));
			conv = (CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

		#else
			layers.PushBack(new LINEAR_LAYER(trainingDigits->GetColumns(), LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new LINEAR_LAYER(layers[layers.GetCount() - 1]->GetOutputSize(), LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new LINEAR_LAYER(layers[layers.GetCount() - 1]->GetOutputSize(), LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
		#endif

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
		layers.PushBack(new ndBrainLayerActivationCategoricalSoftmax(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain.AddLayer(layers[i]);
		}

		brain.InitWeights();
		ndExpandTraceMessage("training mnist database, number of parameters %d\n", brain.GetNumberOfParameters());
	
		SupervisedTrainer optimizer(&brain);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *trainingDigits, *testLabels, *testDigits);
		time = ndGetTimeInMicroseconds() - time;
	
		char path[256];
		#ifdef USE_CONVOLUTIONAL_LAYERS
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
	//ndSharedPtr<ndBrainMatrix> testLabels(LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte"));
	//ndSharedPtr<ndBrainMatrix> testDigits(LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte"));

	if (testLabels && testDigits)
	{
		char path[256];
		#ifdef USE_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif

		ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path));

		ndInt32 numbeOfParam = brain->GetNumberOfParameters();
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		ndExpandTraceMessage("mnist database, number of Parameters %d\n", numbeOfParam);
		if (ndBrainGpuContext::HasGpuSupport())
		//if (0)
		{
			ValidateDataGpu("test data", *(*brain), *testLabels, *testDigits);
		}
		else
		{
			ValidateData("test data", *(*brain), *testLabels, *testDigits);
		}
		time = ndGetTimeInMicroseconds() - time;
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

void ndHandWrittenDigits()
{
	ndSetRandSeed(53);

	//MnistTrainingSet();
	//MnistTestSet();
}
