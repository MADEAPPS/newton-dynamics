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

static void MnistTrainingSet()
{
	//#define USE_CONVOLUTIONAL_LAYERS

	//#define MINIBATCH_BUFFER_SIZE		64
	#define MINIBATCH_BUFFER_SIZE		256
	//#define MINIBATCH_BUFFER_SIZE		2

	#define CONVOLUTIONAL_FEATURE_MAPS		32
	#define MIN_TRAIN_SCORE					0.9999f

	#define NUMBER_OF_EPOCKS				150

	#ifdef USE_CONVOLUTIONAL_LAYERS
		#if 1
			#define CONVOLUTIONAL_LAYER	ndBrainLayerConvolutional_2d
		#else
			#define CONVOLUTIONAL_LAYER	ndBrainLayerConvolutionalWithDropOut_2d
		#endif
	#endif

	#define LINEAR_LAYERS_NEURONS	256
	#define LINEAR_DROPOUT_RATE		ndFloat32 (0.05f)

	#if 0
		#define ACTIVATION_TYPE ndBrainLayerActivationTanh
	#else
		#define ACTIVATION_TYPE ndBrainLayerActivationRelu
		//#define ACTIVATION_TYPE ndBrainLayerActivationElu
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
		SupervisedTrainer(const ndSharedPtr<ndBrain>& brain)
			:ndBrainThreadPool()
			,m_brain(brain)
			,m_context()
			,m_prioritySamples()
			,m_learnRate(ndReal(5.0e-4f))
			,m_miniBatchSize(MINIBATCH_BUFFER_SIZE)
			,m_minCombinedScore(ndInt64(1000000) * ndInt64(1000000))
			,m_minValidationFail(ndInt64(1000000)* ndInt64(1000000))
			,m_hasGpuSupport(m_brain->IsGpuReady())
		{
			ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);

			//threadCount = 1;
			SetThreadCount(threadCount);

			if (m_hasGpuSupport)
			{
				m_context = ndSharedPtr<ndBrainGpuContext>(new ndBrainGpuContext);
				m_trainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainerGpu(m_brain, m_context, m_miniBatchSize));
			}
			else
			{
				ndSharedPtr<ndBrainOptimizerAdamCpu> optimizer(new ndBrainOptimizerAdamCpu);
				m_trainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainerCpu(m_brain, optimizer, this, m_miniBatchSize));
			}
			m_prioritySamples.SetCount(m_miniBatchSize);
		}

		ndInt32 ValidateData(ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			ndInt32 inputSize = testDigits->GetColumns();
			ndInt32 outputSize = testLabels->GetColumns();

			ndBrainVector groundTruth;
			ndBrainVector miniBatchInput;
			ndBrainVector miniBatchOutput;

			groundTruth.SetCount(outputSize * m_miniBatchSize);
			miniBatchInput.SetCount(inputSize * m_miniBatchSize);

			ndInt32 failCount = 0;
			ndInt32 batchesCount = testDigits->GetRows() / m_miniBatchSize;
			ndInt32 batchesSize = batchesCount * m_miniBatchSize;

			for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += m_miniBatchSize)
			{
				for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
				{
					ndBrainMemVector input(&miniBatchInput[i * inputSize], inputSize);
					input.SetCount(inputSize);
					input.Set((*testDigits)[batchStart + i]);
				}
				m_trainer->MakePrediction(miniBatchInput);
				m_trainer->GetOutput(miniBatchOutput);

				for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
				{
					ndBrainMemVector truth(&groundTruth[i * outputSize], outputSize);
					const ndBrainMemVector output(&miniBatchOutput[i * outputSize], outputSize);

					truth.SetCount(outputSize);
					truth.Set((*testLabels)[batchStart + i]);

					ndInt32 maxProbIndex = -1;
					ndBrainFloat maxProbability = ndBrainFloat(-1.0f);
					for (ndInt32 j = 0; j < output.GetCount(); j++)
					{
						if (output[j] > maxProbability)
						{
							maxProbIndex = j;
							maxProbability = output[j];
						}
					}

					ndAssert(maxProbIndex >= 0);
					if (truth[maxProbIndex] == ndReal(0.0f))
					{
						failCount++;
					}
				}
			}
			return failCount;
		}

		void Optimize(ndBrainMatrix* const trainingLabels, ndBrainMatrix* const trainingDigits,
			ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			//// so far best training result on the mnist data set
			//optimizer.SetRegularizer(ndBrainFloat(0.0f));		//         training(100.0%) test(99.35%) 
			////optimizer.SetRegularizer(ndBrainFloat(0.0f));		// dropout training(99.998%) test(99.4%) 
			////optimizer.SetRegularizer(ndBrainFloat(1.0e-5f));	// 
			////optimizer.SetRegularizer(ndBrainFloat(2.0e-5f));	// 
			////optimizer.SetRegularizer(ndBrainFloat(3.0e-5f));	// 
			//optimizer.SetRegularizer(ndBrainFloat(4.0e-5f));	// 

			ndArray<ndUnsigned32> shuffleBuffer;
			for (ndInt32 i = 0; i < trainingDigits->GetCount(); ++i)
			{
				shuffleBuffer.PushBack(ndUnsigned32(i));
			}

			ndInt32 inputSize = m_brain->GetInputSize();
			ndInt32 outputSize = m_brain->GetOutputSize();
			ndBrainTrainer* const trainer = *m_trainer;

			ndBrainVector groundTruth;
			ndBrainVector miniBatchInput;
			ndBrainVector miniBatchOutput;
			ndBrainVector miniBatchOutputGradients;

			groundTruth.SetCount(outputSize * m_miniBatchSize);
			miniBatchInput.SetCount(inputSize * m_miniBatchSize);
			miniBatchOutputGradients.SetCount(outputSize * m_miniBatchSize);
			
			ndInt32 batchesCount = trainingDigits->GetRows() / m_miniBatchSize;
			ndInt32 batchesSize = batchesCount * m_miniBatchSize;

			ndBrainLossCategoricalCrossEntropy loss(outputSize);
			for (ndInt32 epoch = 0; epoch < NUMBER_OF_EPOCKS; ++epoch)
			{
				shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
				for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += m_miniBatchSize)
				{
					for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
					{
						ndUnsigned32 index = shuffleBuffer[batchStart + i];
						ndBrainMemVector input(&miniBatchInput[i * inputSize], inputSize);
						input.SetCount(inputSize);
						input.Set((*trainingDigits)[index]);
					}
					trainer->MakePrediction(miniBatchInput);

					//calculate loss
					trainer->GetOutput(miniBatchOutput);
					for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
					{
						ndUnsigned32 index = shuffleBuffer[batchStart + i];
						ndBrainMemVector grad(&miniBatchOutputGradients[i * outputSize], outputSize);
						const ndBrainMemVector output(&miniBatchOutput[i * outputSize], outputSize);
						const ndBrainMemVector truth(&(*trainingLabels)[index][0], outputSize);
						
						loss.SetTruth(truth);
						loss.GetLoss(output, grad);
					}
					// backpropagate loss.
					trainer->BackPropagate(miniBatchOutputGradients);
					trainer->ApplyLearnRate(m_learnRate);

					#if 0
						//ndBrainVector internalBuffers;
						//ndBrainVector internalParameters;
						//trainer->GetWorkingBuffer(internalBuffers);
						//trainer->GetParameterBuffer(internalParameters);
						//ndInt32 inputSize = trainer->GetBrain()->GetInputSize();
						//ndInt32 outputSize = trainer->GetBrain()->GetOutputSize();
						ndBrainFixSizeVector<1024> xxx1;
						xxx1.SetCount(outputSize);

						trainer->UpdateParameters();
						const ndBrain* const brain = *trainer->GetBrain();
						for (ndInt32 i = 0; i < m_miniBatchSize; i++)
						{
							const ndBrainMemVector in(&miniBatchInput[i * inputSize], inputSize);
							brain->MakePrediction(in, xxx1);
							const ndBrainMemVector xxx0(&miniBatchOutput[i * outputSize], outputSize);
							inputSize *= 1;
						}
					#endif
				}

				ndInt64 testFailCount = ValidateData(testLabels, testDigits) + 1;
				if (testFailCount < m_minValidationFail)
				{
					trainer->UpdateParameters();
					m_minValidationFail = testFailCount + 1;
					ndInt64 trainigFailCount = ValidateData(trainingLabels, trainingDigits) + 1;
					ndInt64 size = trainingLabels->GetCount();
					ndFloat32 score = (ndFloat32)(size - trainigFailCount) / (ndFloat32)size;
					ndExpandTraceMessage("Best model: ");
					ndExpandTraceMessage("  epoch: %d", epoch);
					ndExpandTraceMessage("  success rate:%f%%", score * 100.0f);
					ndExpandTraceMessage("  training fail count:%d", trainigFailCount);
					ndExpandTraceMessage("  test fail count:%d\n", testFailCount);
				} 
				else
				{
					ndInt64 trainigFailCount = ValidateData(trainingLabels, trainingDigits) + 1;
					ndInt64 minCombinedScore = testFailCount * trainigFailCount;
					if (minCombinedScore <= m_minCombinedScore)
					{
						m_minCombinedScore = minCombinedScore;
						ndInt64 size = trainingLabels->GetCount();
						ndFloat32 score = (ndFloat32)(size - trainigFailCount) / (ndFloat32)size;
						ndExpandTraceMessage("  epoch: %d", epoch);
						ndExpandTraceMessage("  success rate:%f%%", score * 100.0f);
						ndExpandTraceMessage("  training fail count:%d", trainigFailCount);
						ndExpandTraceMessage("  test fail count:%d\n", testFailCount);
					}
				}
			}
		}

		ndSharedPtr<ndBrain> m_brain;
		ndSharedPtr<ndBrainGpuContext> m_context;
		ndSharedPtr<ndBrainTrainer> m_trainer;
		ndFixSizeArray<ndFixSizeArray<ndUnsigned32, 16>, MINIBATCH_BUFFER_SIZE> m_prioritySamples;
		ndReal m_learnRate;
		ndInt32 m_miniBatchSize;
		ndInt64 m_minCombinedScore;
		ndInt64 m_minValidationFail;
		bool m_hasGpuSupport;
	};
	
	if (trainingLabels && trainingDigits)
	{
		ndSharedPtr<ndBrain> brain(new ndBrain);
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
			layers.PushBack(new ndBrainLayerLinear(trainingDigits->GetColumns(), LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
			
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
			
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
		#endif

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		layers.PushBack(new ndBrainLayerActivationCategoricalSoftmax(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain->AddLayer(layers[i]);
		}

		brain->InitWeights();
		ndExpandTraceMessage("training mnist database, number of parameters %d\n", brain->GetNumberOfParameters());
	
		SupervisedTrainer optimizer(brain);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *trainingDigits, *testLabels, *testDigits);
		time = ndGetTimeInMicroseconds() - time;
	
		char path[256];
		#ifdef USE_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif
		
		ndBrainSave::Save(*brain, path);

		ndInt32 trainingFailCount = optimizer.ValidateData(*trainingLabels, *trainingDigits);
		ndExpandTraceMessage("training data results:\n");
		ndExpandTraceMessage("  num_right: %d  out of %d\n", trainingLabels->GetCount() - trainingFailCount, trainingLabels->GetCount());
		ndExpandTraceMessage("  num_wrong: %d  out of %d\n", trainingFailCount, trainingLabels->GetCount());
		ndExpandTraceMessage("  success rate %f%%\n", (ndFloat32)(trainingLabels->GetCount() - trainingFailCount) * 100.0f / (ndFloat32)trainingLabels->GetCount());

		ndInt32 testFailCount = optimizer.ValidateData(*testLabels, *testDigits);
		ndExpandTraceMessage("test data results:\n");
		ndExpandTraceMessage("  num_right: %d  out of %d\n", testLabels->GetCount() - testFailCount, testLabels->GetCount());
		ndExpandTraceMessage("  num_wrong: %d  out of %d\n", testFailCount, testLabels->GetCount());
		ndExpandTraceMessage("  success rate %f%%\n", (ndFloat32)(testLabels->GetCount() - testFailCount) * 100.0f / (ndFloat32)testLabels->GetCount());

		ndExpandTraceMessage("treaning time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
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

		//ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path));
		//ndInt32 numbeOfParam = brain->GetNumberOfParameters();
		//ndUnsigned64 time = ndGetTimeInMicroseconds();
		//ndExpandTraceMessage("mnist database, number of Parameters %d\n", numbeOfParam);
		//if (ndBrainGpuContext::HasGpuSupport())
		//{
		//	ValidateDataGpu("test data", *(*brain), *testLabels, *testDigits);
		//}
		//else
		//{
		//	ValidateData("test data", *(*brain), *testLabels, *testDigits);
		//}
		//time = ndGetTimeInMicroseconds() - time;
		//ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

void ndHandWrittenDigits()
{
	ndSetRandSeed(53);
	MnistTrainingSet();
	//MnistTestSet();
}
