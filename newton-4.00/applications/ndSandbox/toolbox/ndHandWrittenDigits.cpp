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

#define MINIST_USE_CPU_TRAINING
 
//#define MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
//#define MNIST_CONVOLUTIONAL_FEATURE_MAPS		32

//#define MINIST_MINIBATCH_BUFFER_SIZE	32
//#define MINIST_MINIBATCH_BUFFER_SIZE	256
#define MINIST_MINIBATCH_BUFFER_SIZE	512
//#define MINIST_MINIBATCH_BUFFER_SIZE	1024

//#define MINIST_LINEAR_LAYERS_NEURONS	64
//#define MINIST_LINEAR_LAYERS_NEURONS	128
//#define MINIST_LINEAR_LAYERS_NEURONS	256
#define MINIST_LINEAR_LAYERS_NEURONS	512
//#define MINIST_LINEAR_LAYERS_NEURONS	1024


//#define MINIST_NUMBER_OF_EPOCHS		70
//#define MINIST_NUMBER_OF_EPOCHS			20
#define MINIST_NUMBER_OF_EPOCHS			1

#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
	#if 1
		#define MINIST_CONVOLUTIONAL_LAYER	ndBrainLayerConvolutional_2d
	#else
		#define MINIST_CONVOLUTIONAL_LAYER	ndBrainLayerConvolutionalWithDropOut_2d
	#endif
#endif

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

class mnistSupervisedTrainer
{
	public:
	mnistSupervisedTrainer(ndSharedPtr<ndBrainContext>& context, const ndSharedPtr<ndBrain>& brain)
		:m_brain(brain)
		,m_bestBrain(ndSharedPtr<ndBrain>(new ndBrain(**brain)))
		,m_learnRate(ndReal(5.0e-4f))
		,m_miniBatchSize(MINIST_MINIBATCH_BUFFER_SIZE)
		,m_minCombinedScore(ndInt64(1000000)* ndInt64(1000000))
		,m_minValidationFail(ndInt64(1000000)* ndInt64(1000000))
	{
		ndTrainerDescriptor descriptor;
		descriptor.m_brain = brain;
		descriptor.m_context = context;
		descriptor.m_learnRate = m_learnRate;
		descriptor.m_minibatchSize = m_miniBatchSize;
		m_trainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
	}

	ndInt32 ValidateData(ndBrainMatrix* const testLabels, const ndSharedPtr<ndBrainFloatBuffer>& data)
	{
		ndInt32 inputSize = m_brain->GetInputSize();
		ndInt32 outputSize = testLabels->GetColumns();

		groundTruth.SetCount(outputSize * m_miniBatchSize);
		miniBatchInput.SetCount(inputSize * m_miniBatchSize);

		ndInt32 failCount = 0;
		ndInt32 batchesCount = testLabels->GetRows() / m_miniBatchSize;
		ndInt32 batchesSize = batchesCount * m_miniBatchSize;

		ndBrainFloatBuffer* const minibatchInputBuffer = m_trainer->GetInputBuffer();
		ndBrainFloatBuffer* const minibatchOutpuBuffer = m_trainer->GetOuputBuffer();

		ndCopyBufferCommandInfo copyDataInfo;
		size_t dataStrideInBytes = inputSize * sizeof(ndReal);
		copyDataInfo.m_dstOffsetInByte = 0;
		copyDataInfo.m_srcOffsetInByte = 0;
		copyDataInfo.m_strideInByte = ndInt32(dataStrideInBytes);
		copyDataInfo.m_srcStrideInByte = ndInt32(dataStrideInBytes);
		copyDataInfo.m_dstStrideInByte = ndInt32(dataStrideInBytes);

		//ndBrainContext* const context = *m_trainer->GetContext();
		for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += m_miniBatchSize)
		{
			//m_trainer->GetContext()->SyncBufferCommandQueue();
			copyDataInfo.m_srcOffsetInByte = ndInt32(batchStart * dataStrideInBytes);
			minibatchInputBuffer->CopyBuffer(copyDataInfo, m_miniBatchSize, **data);

			m_trainer->MakePrediction();
			minibatchOutpuBuffer->VectorFromDevice(miniBatchOutput);

			for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
			{
				ndBrainMemVector truth(&groundTruth[i * outputSize], outputSize);
				truth.SetCount(outputSize);
				truth.Set((*testLabels)[batchStart + i]);
				const ndBrainMemVector output(&miniBatchOutput[i * outputSize], outputSize);

				ndInt64 truthLabel = truth.ArgMax();
				ndInt64 predictedLabel = output.ArgMax();
				failCount += (truthLabel != predictedLabel) ? 1 : 0;
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
		ndBrainContext* const context = *trainer->GetContext();

		ndBrainVector weightAndBias;
		ndBrainVector miniBatchOutputGradients;

		groundTruth.SetCount(outputSize * m_miniBatchSize);
		miniBatchInput.SetCount(inputSize * m_miniBatchSize);
		miniBatchOutput.SetCount(outputSize * m_miniBatchSize);
		miniBatchOutputGradients.SetCount(outputSize * m_miniBatchSize);

		ndInt32 batchesCount = trainingLabels->GetRows() / m_miniBatchSize;
		ndInt32 batchesSize = batchesCount * m_miniBatchSize;

		m_bestBrain = ndSharedPtr<ndBrain>(new ndBrain(**trainer->GetBrain()));

		ndBrainLossCategoricalCrossEntropy loss(outputSize);
		ndBrainFloatBuffer* const minibatchInputBuffer = m_trainer->GetInputBuffer();
		ndBrainFloatBuffer* const minibatchOutpuBuffer = m_trainer->GetOuputBuffer();
		ndBrainFloatBuffer* const weightdAndBiasBuffer = m_trainer->GetWeightAndBiasBuffer();
		ndBrainFloatBuffer* const minibatchOutpuGradientBuffer = m_trainer->GetOuputGradientBuffer();

		ndCopyBufferCommandInfo copyDataInfo;
		ndInt32 dataStrideInBytes = ndInt32(inputSize * sizeof(ndReal));
		copyDataInfo.m_dstOffsetInByte = 0;
		copyDataInfo.m_srcOffsetInByte = 0;
		copyDataInfo.m_strideInByte = dataStrideInBytes;
		copyDataInfo.m_srcStrideInByte = dataStrideInBytes;
		copyDataInfo.m_dstStrideInByte = dataStrideInBytes;

		ndCopyBufferCommandInfo copyLabelsInfo;
		ndInt32 labelsStrideInBytes = ndInt32(outputSize * sizeof(ndReal));
		copyLabelsInfo.m_dstOffsetInByte = 0;
		copyLabelsInfo.m_srcOffsetInByte = 0;
		copyLabelsInfo.m_strideInByte = labelsStrideInBytes;
		copyLabelsInfo.m_srcStrideInByte = labelsStrideInBytes;
		copyLabelsInfo.m_dstStrideInByte = labelsStrideInBytes;

		ndCopyBufferCommandInfo copyIndicesInfo;
		ndInt32 copyIndicesStrideInBytes = ndInt32(m_miniBatchSize * sizeof(ndInt32));
		copyIndicesInfo.m_dstOffsetInByte = 0;
		copyIndicesInfo.m_srcOffsetInByte = 0;
		copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;

		ndBrainFloatBuffer groundTruthMinibatch(*minibatchOutpuBuffer);
		ndBrainIntegerBuffer randomShuffleBuffer(context, shuffleBuffer.GetCount());

		for (ndInt32 epoch = 0; epoch < MINIST_NUMBER_OF_EPOCHS; ++epoch)
		{
			shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
			randomShuffleBuffer.MemoryToDevice(0, shuffleBuffer.GetCount() * sizeof(ndInt32), &shuffleBuffer[0]);

			for (ndInt32 batchStart = 0; batchStart < batchesSize; batchStart += m_miniBatchSize)
			{
				// wait until previous epoch is completed
				context->SyncBufferCommandQueue();
				copyIndicesInfo.m_srcOffsetInByte = ndInt32 (batchStart * sizeof(ndUnsigned32));
				m_indirectMiniBatch->CopyBuffer(copyIndicesInfo, 1, randomShuffleBuffer);

				minibatchInputBuffer->CopyBufferIndirect(copyDataInfo, **m_indirectMiniBatch, **m_trainingData);
				groundTruthMinibatch.CopyBufferIndirect(copyLabelsInfo, **m_indirectMiniBatch, **m_trainingLabels);
				trainer->MakePrediction();

				//calculate loss
#if 0
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
				minibatchOutpuGradientBuffer->VectorToDevice(miniBatchOutputGradients);
#else
				//for non categorical soft max, calculate the least scuare error lost
				//minibatchOutpuGradientBuffer->CopyBuffer(*minibatchOutpuBuffer);
				//context->Sub(*minibatchOutpuGradientBuffer, **groundTruthMinibatch);

				//for categorical soft max, just pass the categorical class as gradient loss
				minibatchOutpuGradientBuffer->CopyBuffer(groundTruthMinibatch);
#endif
				// back propagate loss.
				trainer->BackPropagate();
				trainer->ApplyLearnRate();
			}

#if 1
			// do a detail validation, by counting all the fail in test and tranning set
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

				ndExpandTraceMessage("epoch: %d\n", epoch);
				ndExpandTraceMessage("  saving best model:");
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

					ndExpandTraceMessage("epoch: %d\n", epoch);
					ndExpandTraceMessage("  success rate:%f%%", score * 100.0f);
					ndExpandTraceMessage("  training fail count:%d", trainigFailCount);
					ndExpandTraceMessage("  test fail count:%d\n", testFailCount);
				}
			}
#else
			ndInt64 testFailCount = ValidateData(testLabels, m_testData);
			if (testFailCount < m_minValidationFail)
			{
				weightdAndBiasBuffer->VectorFromDevice(weightAndBias);
				trainer->UpdateParameters(weightAndBias);
				m_bestBrain->CopyFrom(**trainer->GetBrain());
				m_minValidationFail = testFailCount;

				ndExpandTraceMessage("epoch: %d\n", epoch);
				ndExpandTraceMessage("   best model: test fail count:%d\n", testFailCount);
			}
#endif
		}
		trainer->GetBrain()->CopyFrom(**m_bestBrain);
	}

	ndSharedPtr<ndBrain> m_brain;
	ndSharedPtr<ndBrain> m_bestBrain;
	ndSharedPtr<ndBrainTrainer> m_trainer;

	ndSharedPtr<ndBrainFloatBuffer> m_testData;
	ndSharedPtr<ndBrainFloatBuffer> m_testLabels;
	ndSharedPtr<ndBrainFloatBuffer> m_trainingData;
	ndSharedPtr<ndBrainFloatBuffer> m_trainingLabels;
	ndSharedPtr<ndBrainIntegerBuffer> m_indirectMiniBatch;

	ndBrainVector groundTruth;
	ndBrainVector miniBatchInput;
	ndBrainVector miniBatchOutput;

	ndReal m_learnRate;
	ndInt32 m_miniBatchSize;
	ndInt64 m_minCombinedScore;
	ndInt64 m_minValidationFail;
};

static void MnistTrainingSet()
{
	ndSharedPtr<ndBrainMatrix> testLabels(LoadMnistLabelData("mnistDatabase/t10k-labels.idx1-ubyte"));
	ndSharedPtr<ndBrainMatrix> trainingLabels (LoadMnistLabelData("mnistDatabase/train-labels.idx1-ubyte"));

	ndSharedPtr<ndBrainMatrix> testDigits(LoadMnistSampleData("mnistDatabase/t10k-images.idx3-ubyte"));
	ndSharedPtr<ndBrainMatrix> trainingDigits (LoadMnistSampleData("mnistDatabase/train-images.idx3-ubyte"));
	
	if (trainingLabels && trainingDigits)
	{
		ndSharedPtr<ndBrain> brain(new ndBrain);
		ndFixSizeArray<ndBrainLayer*, 32> layers;

		#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
			ndAssert(0);
			//ndInt32 height = 28;
			//ndInt32 width = trainingDigits->GetColumns() / height;
			//ndAssert((height * width) == trainingDigits->GetColumns());
			//
			//const MINIST_CONVOLUTIONAL_LAYER* conv;
			//const ndBrainLayerImagePolling_2x2* pooling;
			//
			//layers.PushBack(new MINIST_CONVOLUTIONAL_LAYER(width, height, 1, 3, MNIST_CONVOLUTIONAL_FEATURE_MAPS));
			//conv = (MINIST_CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			//layers.PushBack(new MINIST_ACTIVATION_TYPE(conv->GetOutputSize()));
			//layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			//pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
			//
			//layers.PushBack(new MINIST_CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, MNIST_CONVOLUTIONAL_FEATURE_MAPS));
			//conv = (MINIST_CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			//layers.PushBack(new MINIST_ACTIVATION_TYPE(conv->GetOutputSize()));
			//layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			//pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
			//
			//layers.PushBack(new MINIST_CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, MNIST_CONVOLUTIONAL_FEATURE_MAPS));
			//conv = (MINIST_CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
			//layers.PushBack(new MINIST_ACTIVATION_TYPE(conv->GetOutputSize()));
			//layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			//pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			//layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
			//layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
			//layers.PushBack(new ndBrainLayerActivationCategoricalSoftmax(layers[layers.GetCount() - 1]->GetOutputSize()));

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

			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
			layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
			layers.PushBack(new ndBrainLayerActivationCategoricalSoftmax(layers[layers.GetCount() - 1]->GetOutputSize()));
		#endif

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain->AddLayer(layers[i]);
		}

		brain->InitWeights();

		bool isGpuReady = brain->IsGpuReady();
		#ifdef MINIST_USE_CPU_TRAINING
			isGpuReady = false;
		#endif

		ndSharedPtr<ndBrainContext> context(isGpuReady ? (ndBrainContext*)new ndBrainGpuContext : (ndBrainContext*)new ndBrainCpuContext);
		mnistSupervisedTrainer trainer(context, brain);

		trainer.m_testData = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*context, **testDigits));
		trainer.m_testLabels = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*context, **testLabels));
		trainer.m_trainingData = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*context, **trainingDigits));
		trainer.m_trainingLabels = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*context, **trainingLabels));
		trainer.m_indirectMiniBatch = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*context, trainer.m_miniBatchSize));

		ndUnsigned64 time = ndGetTimeInMicroseconds();
		trainer.Optimize(*trainingLabels, *testLabels);
		time = ndGetTimeInMicroseconds() - time;

		char path[256];
		#ifdef MNIST_USE_MINIST_CONVOLUTIONAL_LAYERS
		ndGetWorkingFileName("mnistDatabase/mnist-cnn.dnn", path);
		#else
		ndGetWorkingFileName("mnistDatabase/mnist.dnn", path);
		#endif
		
		ndBrainSave::Save(*trainer.m_bestBrain, path);
		
		mnistSupervisedTrainer inference(context, trainer.m_bestBrain);
		ndInt32 testFailCount = inference.ValidateData(*testLabels, trainer.m_testData);
		ndInt32 trainingFailCount = inference.ValidateData(*trainingLabels, trainer.m_trainingData);
		context->SyncBufferCommandQueue();
		
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
	//MnistTrainingSet();
	//MnistTestSet();
}
