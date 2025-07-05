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

//#define MINIST_USE_CPU_TRAINING
 
//#define MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS

#define MINIST_MINIBATCH_BUFFER_SIZE			512
//#define MINIST_MINIBATCH_BUFFER_SIZE			(1 * ND_CPU_MINI_BATCH_SIZE_GRANULARITY)

#define MNIST_CONVOLUTIONAL_FEATURE_MAPS		32
//#define MIN_TRAIN_SCORE						0.9999f

//#define MINIST_NUMBER_OF_EPOCHS				70
#define MINIST_NUMBER_OF_EPOCHS					20
//#define MINIST_NUMBER_OF_EPOCHS					1

#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
	#if 1
		#define MINIST_CONVOLUTIONAL_LAYER	ndBrainLayerConvolutional_2d
	#else
		#define MINIST_CONVOLUTIONAL_LAYER	ndBrainLayerConvolutionalWithDropOut_2d
	#endif
#endif

//#define MINIST_LINEAR_LAYERS_NEURONS	256
#define MINIST_LINEAR_LAYERS_NEURONS	512
//#define MINIST_LINEAR_LAYERS_NEURONS	1024

#define MINIST_LINEAR_DROPOUT_RATE		ndFloat32 (0.05f)

#define MINIST_ACTIVATION_TYPE ndBrainLayerActivationRelu
//#define MINIST_ACTIVATION_TYPE ndBrainLayerActivationElu
//#define MINIST_ACTIVATION_TYPE ndBrainLayerActivationSigmoidLinear

//#if 1
//	//#define MINIST_DIGIT_FILTER_LAYER_TYPE ndBrainLayerConvolutional_2d
//	#define MINIST_DIGIT_FILTER_LAYER_TYPE ndBrainLayerConvolutionalWithDropOut_2d
//#else
//	#define MINIST_DIGIT_FILTER_LAYER_TYPE ndBrainLayerCrossCorrelation_2d			
//#endif


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
	ndSharedPtr<ndBrainMatrix> testLabels(LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> trainingLabels (LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte"));

	ndSharedPtr<ndBrainMatrix> testDigits(LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte"));
	ndSharedPtr<ndBrainMatrix> trainingDigits (LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte"));
	
	class SupervisedTrainer
	{
		public:
		SupervisedTrainer(ndSharedPtr<ndBrainContext>& context, const ndSharedPtr<ndBrain>& brain)
			:m_brain(brain)
			,m_bestBrain(ndSharedPtr<ndBrain>(new ndBrain(**brain)))
			,m_learnRate(ndReal(5.0e-4f))
			,m_miniBatchSize(MINIST_MINIBATCH_BUFFER_SIZE)
			,m_minCombinedScore(ndInt64(1000000) * ndInt64(1000000))
			,m_minValidationFail(ndInt64(1000000) * ndInt64(1000000))
		{
			ndTrainerDescriptor descritor;
			descritor.m_brain = brain;
			descritor.m_context = context;
			descritor.m_learnRate = m_learnRate;
			descritor.m_minibatchSize = m_miniBatchSize;
			m_trainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descritor));
		}

		ndInt32 ValidateData(ndBrainMatrix* const testLabels, const ndSharedPtr<ndBrainFloatBuffer>& data)
		{
			//return 0;
			ndBrainVector groundTruth;
			ndBrainVector miniBatchInput;
			ndBrainVector miniBatchOutput;
			
			ndInt32 inputSize = m_brain->GetInputSize();
			ndInt32 outputSize = testLabels->GetColumns();
			
			groundTruth.SetCount(outputSize * m_miniBatchSize);
			miniBatchInput.SetCount(inputSize * m_miniBatchSize);
			
			ndInt32 failCount = 0;
			ndInt32 batchesCount = testLabels->GetRows() / m_miniBatchSize;
			ndInt32 batchesSize = batchesCount * m_miniBatchSize;
			
			size_t strideInBytes = inputSize * sizeof(ndReal);
			ndBrainFloatBuffer* const minibatchInputBuffer = m_trainer->GetInputBuffer();
			ndBrainFloatBuffer* const minibatchOutpuBuffer = m_trainer->GetOuputBuffer();
			
			ndCopyBufferCommandInfo copyBufferInfo;
			copyBufferInfo.m_dstOffsetInByte = 0;
			copyBufferInfo.m_srcOffsetInByte = 0;
			copyBufferInfo.m_strideInByte = ndInt32(strideInBytes);
			copyBufferInfo.m_srcStrideInByte = ndInt32(strideInBytes);
			copyBufferInfo.m_dstStrideInByte = ndInt32(strideInBytes);
			for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += m_miniBatchSize)
			{
				copyBufferInfo.m_srcOffsetInByte = ndInt32 (batchStart * strideInBytes);
				minibatchInputBuffer->CopyBuffer(copyBufferInfo, m_miniBatchSize, **data);
			
				m_trainer->MakePrediction();
				//trainer->GetContext()->SyncBufferCommandQueue();
				minibatchOutpuBuffer->VectorFromDevice(miniBatchOutput);
				
				for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
				{
					ndBrainMemVector truth(&groundTruth[i * outputSize], outputSize);
					truth.SetCount(outputSize);
					truth.Set((*testLabels)[batchStart + i]);
				
					ndInt32 maxProbIndex = -1;
					ndBrainFloat maxProbability = ndBrainFloat(-1.0f);
				
					const ndBrainMemVector output(&miniBatchOutput[i * outputSize], outputSize);
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

		void Optimize(ndBrainMatrix* const trainingLabels, ndBrainMatrix* const testLabels)
		{
			//// so far best training result on the mnist data set
			//optimizer.SetRegularizer(ndBrainFloat(0.0f));		//         training(100.0%) test(99.35%) 
			////optimizer.SetRegularizer(ndBrainFloat(0.0f));		// dropout training(99.998%) test(99.4%) 
			////optimizer.SetRegularizer(ndBrainFloat(1.0e-5f));	// 
			////optimizer.SetRegularizer(ndBrainFloat(2.0e-5f));	// 
			////optimizer.SetRegularizer(ndBrainFloat(3.0e-5f));	// 
			//optimizer.SetRegularizer(ndBrainFloat(4.0e-5f));	// 
			
			ndArray<ndUnsigned32> shuffleBuffer;
			for (ndInt32 i = 0; i < trainingLabels->GetCount(); ++i)
			{
				shuffleBuffer.PushBack(ndUnsigned32(i));
			}
			
			ndInt32 inputSize = m_brain->GetInputSize();
			ndInt32 outputSize = m_brain->GetOutputSize();
			ndBrainTrainer* const trainer = *m_trainer;
			
			ndBrainVector groundTruth;
			ndBrainVector weightAndBias;
			ndBrainVector miniBatchInput;
			ndBrainVector miniBatchOutput;
			ndBrainVector miniBatchOutputGradients;
			
			groundTruth.SetCount(outputSize * m_miniBatchSize);
			miniBatchInput.SetCount(inputSize * m_miniBatchSize);
			miniBatchOutput.SetCount(outputSize * m_miniBatchSize);
			miniBatchOutputGradients.SetCount(outputSize * m_miniBatchSize);
			
			ndInt32 batchesCount = trainingLabels->GetRows() / m_miniBatchSize;
			ndInt32 batchesSize = batchesCount * m_miniBatchSize;
			
			m_bestBrain = ndSharedPtr<ndBrain>(new ndBrain (**trainer->GetBrain()));
			
			ndBrainLossCategoricalCrossEntropy loss(outputSize);
			
			size_t strideInBytes = size_t(inputSize * sizeof(ndReal));
			ndBrainFloatBuffer* const minibatchInputBuffer = m_trainer->GetInputBuffer();
			ndBrainFloatBuffer* const minibatchOutpuBuffer = m_trainer->GetOuputBuffer();
			ndBrainFloatBuffer* const weightdAndBiasBuffer = m_trainer->GetWeightAndBiasBuffer();
			ndBrainFloatBuffer* const minibatchOutpuGradientBuffer = m_trainer->GetOuputGradientBuffer();
			
			ndCopyBufferCommandInfo copyBuffer;
			copyBuffer.m_dstOffsetInByte = 0;
			copyBuffer.m_srcOffsetInByte = 0;
			copyBuffer.m_strideInByte = ndInt32(strideInBytes);
			copyBuffer.m_srcStrideInByte = ndInt32(strideInBytes);
			copyBuffer.m_dstStrideInByte = ndInt32(strideInBytes);

			for (ndInt32 epoch = 0; epoch < MINIST_NUMBER_OF_EPOCHS; ++epoch)
			{
				shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
				for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += m_miniBatchSize)
				{
					m_indirectMiniBatch->MemoryToDevice(0, m_miniBatchSize * sizeof(ndUnsigned32), &shuffleBuffer[batchStart]);
					minibatchInputBuffer->CopyBufferIndirect(copyBuffer, **m_indirectMiniBatch, **m_trainingData);
					trainer->MakePrediction();
					
					//calculate loss
					//trainer->GetContext()->SyncBufferCommandQueue();
					minibatchOutpuBuffer->VectorFromDevice(miniBatchOutput);

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
					minibatchOutpuGradientBuffer->VectorToDevice(miniBatchOutputGradients);
					trainer->BackPropagate();
					trainer->ApplyLearnRate(); 
				}

				ndInt64 testFailCount = ValidateData(testLabels, m_testData) + 1;
				if (testFailCount < m_minValidationFail)
				{
					weightdAndBiasBuffer->VectorFromDevice(weightAndBias);
					trainer->UpdateParameters(weightAndBias);
					m_bestBrain->CopyFrom(**trainer->GetBrain());
				
					m_minValidationFail = testFailCount + 1;
					ndInt64 trainigFailCount = ValidateData(trainingLabels, m_trainingData) + 1;
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
					ndInt64 trainigFailCount = ValidateData(trainingLabels, m_trainingData) + 1;
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
			trainer->GetBrain()->CopyFrom(**m_bestBrain);
		}

		ndSharedPtr<ndBrain> m_brain;
		ndSharedPtr<ndBrain> m_bestBrain;
		ndSharedPtr<ndBrainTrainer> m_trainer;

		ndSharedPtr<ndBrainFloatBuffer> m_testData;
		ndSharedPtr<ndBrainFloatBuffer> m_trainingData;
		ndSharedPtr<ndBrainIntegerBuffer> m_indirectMiniBatch;
		ndReal m_learnRate;
		ndInt32 m_miniBatchSize;
		ndInt64 m_minCombinedScore;
		ndInt64 m_minValidationFail;
	};
	
	if (trainingLabels && trainingDigits)
	{
		ndSharedPtr<ndBrain> brain(new ndBrain);
		ndFixSizeArray<ndBrainLayer*, 32> layers;

		#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
			ndInt32 height = 28;
			ndInt32 width = trainingDigits->GetColumns() / height;
			ndAssert((height * width) == trainingDigits->GetColumns());

			const MINIST_CONVOLUTIONAL_LAYER* conv;
			const ndBrainLayerImagePolling_2x2* pooling;

			layers.PushBack(new MINIST_CONVOLUTIONAL_LAYER(width, height, 1, 3, MNIST_CONVOLUTIONAL_FEATURE_MAPS));
			conv = (MINIST_CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new MINIST_ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			layers.PushBack(new MINIST_CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, MNIST_CONVOLUTIONAL_FEATURE_MAPS));
			conv = (MINIST_CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new MINIST_ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			layers.PushBack(new MINIST_CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, MNIST_CONVOLUTIONAL_FEATURE_MAPS));
			conv = (MINIST_CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new MINIST_ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

		#else
			layers.PushBack(new ndBrainLayerLinear(trainingDigits->GetColumns(), MINIST_LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new MINIST_ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
			
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), MINIST_LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new MINIST_ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
			 
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), MINIST_LINEAR_LAYERS_NEURONS));
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new MINIST_ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
		#endif

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		layers.PushBack(new ndBrainLayerActivationCategoricalSoftmax(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain->AddLayer(layers[i]);
		}

		brain->InitWeights();

		bool isGpuReady = brain->IsGpuReady();
#ifdef MINIST_USE_CPU_TRAINING
		isGpuReady = false;
#else
		ndAssert(isGpuReady);
#endif

		ndSharedPtr<ndBrainContext> context(isGpuReady ? (ndBrainContext*)new ndBrainGpuContext : (ndBrainContext*)new ndBrainCpuContext);
		SupervisedTrainer optimizer(context, brain);

		optimizer.m_testData = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*context, **testDigits));
		optimizer.m_trainingData = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*context, **trainingDigits));
		optimizer.m_indirectMiniBatch = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*context, optimizer.m_miniBatchSize, true));

		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *testLabels);
		time = ndGetTimeInMicroseconds() - time;

		char path[256];
		#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif
		
		ndBrainSave::Save(*optimizer.m_bestBrain, path);
		
		SupervisedTrainer inference(context, optimizer.m_bestBrain);
		ndInt32 testFailCount = inference.ValidateData(*testLabels, optimizer.m_testData);
		ndInt32 trainingFailCount = inference.ValidateData(*trainingLabels, optimizer.m_trainingData);
		
		ndExpandTraceMessage("\n");
		ndExpandTraceMessage("results:\n");
		ndExpandTraceMessage("mnist database, model number of parameters %d\n", brain->GetNumberOfParameters());
		ndExpandTraceMessage("training time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
		
		// training data report
		ndExpandTraceMessage("training data results:\n");
		ndExpandTraceMessage("  num_right: %d  out of %d\n", trainingLabels->GetCount() - trainingFailCount, trainingLabels->GetCount());
		ndExpandTraceMessage("  num_wrong: %d  out of %d\n", trainingFailCount, trainingLabels->GetCount());
		ndExpandTraceMessage("  success rate %f%%\n", (ndFloat32)(trainingLabels->GetCount() - trainingFailCount) * 100.0f / (ndFloat32)trainingLabels->GetCount());
		
		// test data report
		ndExpandTraceMessage("test data results:\n");
		ndExpandTraceMessage("  num_right: %d  out of %d\n", testLabels->GetCount() - testFailCount, testLabels->GetCount());
		ndExpandTraceMessage("  num_wrong: %d  out of %d\n", testFailCount, testLabels->GetCount());
		ndExpandTraceMessage("  success rate %f%%\n", (ndFloat32)(testLabels->GetCount() - testFailCount) * 100.0f / (ndFloat32)testLabels->GetCount());
	}
}

static void MnistTestSet()
{
	ndSharedPtr<ndBrainMatrix> testLabels (LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> testDigits (LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte"));

	if (testLabels && testDigits)
	{
		char path[256];
		#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif

		ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path));
		ndInt32 numbeOfParam = brain->GetNumberOfParameters();

		ndInt32 minibatchSize = 256;
		ndSharedPtr<ndBrainContext> context(new ndBrainCpuContext);
		ndTrainerDescriptor descriptor(brain, context, minibatchSize, 0);
		ndSharedPtr<ndBrainTrainerInference> inference(ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(descriptor)));
		
		ndBrainVector groundTruth;
		ndBrainVector miniBatchInput;
		ndBrainVector miniBatchOutput;
		
		ndInt32 inputSize = testDigits->GetColumns();
		ndInt32 outputSize = testLabels->GetColumns();
		
		miniBatchInput.SetCount(inputSize * minibatchSize);
		groundTruth.SetCount(outputSize * minibatchSize);
		
		ndUnsigned64 time = ndGetTimeInMicroseconds();

		ndInt32 failCount = 0;
		ndInt32 batchesCount = testDigits->GetRows() / minibatchSize;
		ndInt32 batchesSize = batchesCount * minibatchSize;

		ndBrainFloatBuffer* const minibatchInputBuffer = inference->GetInputBuffer();
		ndBrainFloatBuffer* const minibatchOutpuBuffer = inference->GetOuputBuffer();

		for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += minibatchSize)
		{
			for (ndInt32 i = 0; i < minibatchSize; ++i)
			{
				ndBrainMemVector input(&miniBatchInput[i * inputSize], inputSize);
				input.SetCount(inputSize);
				input.Set((**testDigits)[batchStart + i]);
			}
			
			minibatchInputBuffer->VectorToDevice(miniBatchInput);
			inference->MakePrediction();
			minibatchOutpuBuffer->VectorFromDevice(miniBatchOutput);
		
			for (ndInt32 i = 0; i < minibatchSize; ++i)
			{
				ndBrainMemVector truth(&groundTruth[i * outputSize], outputSize);
				const ndBrainMemVector output(&miniBatchOutput[i * outputSize], outputSize);
		
				truth.SetCount(outputSize);
				truth.Set((**testLabels)[batchStart + i]);
		
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
		
		time = ndGetTimeInMicroseconds() - time;
		
		ndFloat32 score = (ndFloat32)(batchesSize - failCount) / (ndFloat32)batchesSize;
		ndExpandTraceMessage("mnist database, number of Parameters %d\n", numbeOfParam);
		ndExpandTraceMessage("  success rate:%f%%\n", score * 100.0f);
		ndExpandTraceMessage("  test fail count:%d\n", failCount);
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

void ndHandWrittenDigits()
{
	ndSetRandSeed(53);
	MnistTrainingSet();
	//MnistTestSet();
}
