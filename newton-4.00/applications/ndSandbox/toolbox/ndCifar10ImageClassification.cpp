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

static void LoadTrainingData(ndSharedPtr<ndBrainMatrix>& trainingImages, ndSharedPtr<ndBrainMatrix>& srcTrainingImages, ndSharedPtr<ndBrainMatrix>& trainingLabels)
{
	const ndInt32 batches = 5;
	const ndInt32 pixelSize = 32 * 32;
	//char outPathName[1024];
	ndUnsigned8 data[pixelSize * 3];

	trainingLabels = new ndBrainMatrix(ndInt32(batches * 10000), ndInt32(10));
	trainingImages = new ndBrainMatrix(ndInt32(batches * 10000), ndInt32(pixelSize * 3));
	srcTrainingImages = new ndBrainMatrix(ndInt32(batches * 10000), ndInt32(pixelSize * 3));

	ndBrainMatrix& labelMatrix = *(*trainingLabels);
	ndBrainMatrix& imageMatrix = *(*trainingImages);
	ndBrainMatrix& srcImageMatrix = *(*srcTrainingImages);

	ndInt32 base = 0;
	labelMatrix.Set(ndBrainFloat(0.0f));
	char filename[1024];
	for (ndInt32 i = 0; i < batches; ++i)
	{
		snprintf(filename, sizeof (filename), "cifar-10-batches-bin/data_batch_%d.bin", i + 1);
		//ndGetWorkingFileName(filename, outPathName);
		const ndString outPathName(ndGetWorkingFileName(filename));
		FILE* const fp = fopen(outPathName.GetStr(), "rb");
		if (fp)
		{
			for (ndInt32 j = 0; j < 10000; ++j)
			{
				size_t ret = 0;
				ndInt32 label = 0;
				ret = fread(&label, 1, 1, fp);

				labelMatrix[j + base][label] = ndBrainFloat(1.0f);
				ndBrainVector& image = imageMatrix[j + base];
				ndBrainVector& src = srcImageMatrix[j + base];
				ret = fread(data, 1, pixelSize * 3, fp);
				for (ndInt32 k = 0; k < pixelSize * 3; ++k)
				{
					image[k] = ndBrainFloat(data[k]) / ndBrainFloat(255.0f);
					src[k] = image[k];
				}

				for (ndInt32 k = 0; k < 3; ++k)
				{
					ndBrainMemVector imageChannel (&image[k * pixelSize], pixelSize);
					imageChannel.GaussianNormalize();
				}
			}
			base += 10000;
			fclose(fp);
		}
	}
}

static void SaveImage(const ndBrainVector& input, const char* const name)
{
	unsigned char pBits[32][32][3];

	static ndInt32 index = 0;
	const ndBrainFloat* src = &input[0];
	for (ndInt32 y = 0; y < 32; ++y)
	{
		for (ndInt32 x = 0; x < 32; ++x)
		{
			pBits[y][x][0] = (unsigned char)(src[0 * 32 * 32] * 255.0f);
			pBits[y][x][1] = (unsigned char)(src[1 * 32 * 32] * 255.0f);
			pBits[y][x][2] = (unsigned char)(src[2 * 32 * 32] * 255.0f);
			src ++;
		}
	}

	char name1[256];
	snprintf(name1, sizeof (name1), "%s_%d.png", name, index);
	index++;
	lodepng_encode_file(name1, &pBits[0][0][0], 32, 32, LCT_RGB, 8);
}

static void LoadTestData(ndSharedPtr<ndBrainMatrix>& images, ndSharedPtr<ndBrainMatrix>& srcImages, ndSharedPtr<ndBrainMatrix>& labels)
{
	const ndInt32 pixelSize = 32 * 32;
	//char outPathName[1024];
	ndUnsigned8 data[pixelSize * 3];

	labels = new ndBrainMatrix(ndInt32(10000), ndInt32(10));
	images = new ndBrainMatrix(ndInt32(10000), ndInt32(pixelSize * 3));
	srcImages = new ndBrainMatrix(ndInt32(10000), ndInt32(pixelSize * 3));

	ndBrainMatrix& labelMatrix = *(*labels);
	ndBrainMatrix& imageMatrix = *(*images);
	ndBrainMatrix& srcImageMatrix = *(*srcImages);

	labelMatrix.Set(ndBrainFloat(0.0f));
	//ndGetWorkingFileName("cifar-10-batches-bin/test_batch.bin", outPathName);
	const ndString outPathName(ndGetWorkingFileName("cifar-10-batches-bin/test_batch.bin"));
	FILE* const fp = fopen(outPathName.GetStr(), "rb");
	if (fp)
	{
		for (ndInt32 j = 0; j < 10000; ++j)
		{
			size_t ret = 0;
			ndInt32 label = 0;
			ret = fread(&label, 1, 1, fp);

			labelMatrix[j][label] = ndBrainFloat(1.0f);
			ndBrainVector& image = imageMatrix[ndInt32(j)];
			ndBrainVector& src = srcImageMatrix[ndInt32(j)];
			ret = fread(data, 1, pixelSize * 3, fp);
			for (ndInt32 k = 0; k < pixelSize * 3; ++k)
			{
				image[k] = ndBrainFloat(data[k]) / ndBrainFloat(255.0f);
				src[k] = image[k];
			}

			for (ndInt32 k = 0; k < 3; ++k)
			{
				ndBrainMemVector imageChannel(&image[k * pixelSize], pixelSize);
				imageChannel.GaussianNormalize();
			}
		}
		fclose(fp);
	}
}

static void ValidateData(const char* const title, ndBrain& brain, ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits, ndBrainMatrix* const srcTestDigits)
{
	ndBrainVector output;
	output.SetCount((*testLabels)[0].GetCount());

	const char* categories[] = {
		"airplane",
		"automobile",
		"bird",
		"cat",
		"deer",
		"dog",
		"frog",
		"horse",
		"ship",
		"truck" };


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
		if (truth[index] == ndReal(0.0f))
		{
			const ndBrainVector& src = (*srcTestDigits)[i];
			SaveImage(src, categories[index]);
			failCount++;
		}
	}
	ndExpandTraceMessage("%s\n", title);
	ndExpandTraceMessage("num_right: %d  out of %d\n", testDigits->GetCount() - failCount, testDigits->GetCount());
	ndExpandTraceMessage("num_wrong: %d  out of %d\n", failCount, testDigits->GetCount());
	ndExpandTraceMessage("success rate %f%%\n", (ndFloat32)(testDigits->GetCount() - failCount) * 100.0f / (ndFloat32)testDigits->GetCount());
}

static void Cifar10TrainingSet()
{
	#define MINI_BATCH_BUFFER_SIZE	64

	#if 1
		#define CONVOLUTIONAL_LAYER	ndBrainLayerConvolutional_2d
	#else
		#define CONVOLUTIONAL_LAYER	ndBrainLayerConvolutionalWithDropOut_2d
	#endif

	#if 1
		#define LINEAR_LAYER_NEURONS	128
		#define LINEAR_LAYER			ndBrainLayerLinear
	#else
		#define LINEAR_LAYER_NEURONS	128
		#define LINEAR_LAYER			ndBrainLayerLinearWithDropOut
	#endif

	#if 1
		#define ACTIVATION_TYPE	ndBrainLayerActivationRelu
	#else
		#define ACTIVATION_TYPE	ndBrainLayerActivationTanh
	#endif

	ndSharedPtr<ndBrainMatrix> testLabels;
	ndSharedPtr<ndBrainMatrix> testImages;
	ndSharedPtr<ndBrainMatrix> srcTestImages;
	ndSharedPtr<ndBrainMatrix> trainingLabels;
	ndSharedPtr<ndBrainMatrix> trainingImages;
	ndSharedPtr<ndBrainMatrix> srcTrainingImages;

	LoadTestData(testImages, srcTestImages, testLabels);
	LoadTrainingData(trainingImages, srcTrainingImages, trainingLabels);

	class SupervisedTrainer : public ndBrainThreadPool
	{
		public:
		SupervisedTrainer(ndBrain* const brain)
			:ndBrainThreadPool()
			,m_brain(*brain)
			,m_learnRate(ndReal(5.0e-4f))
			,m_miniBatchSize(MINI_BATCH_BUFFER_SIZE)
		{
			ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);
			//threadCount = 1;
			SetThreadCount(threadCount);
ndAssert(0);
#if 0
			for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
			{
				ndBrainTrainer* const trainer = new ndBrainTrainer(&m_brain);
				m_trainers.PushBack(trainer);
			}
#endif
		}

		~SupervisedTrainer()
		{
			for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
			{
				delete m_trainers[i];
			}
		}

		//void Optimize(ndBrainMatrix* const trainingLabels, const ndBrainMatrix* const sourceTrainingImages,
		//			  ndBrainMatrix* const testLabels, ndBrainMatrix* const testImages)
		void Optimize(ndBrainMatrix* const, const ndBrainMatrix* const,
					  ndBrainMatrix* const, ndBrainMatrix* const)

		{
			ndAssert(0);
#if 0
			ndUnsigned32 failCount[D_MAX_THREADS_COUNT];
			ndUnsigned32 miniBatchArray[MINI_BATCH_BUFFER_SIZE];

			ndAtomic<ndInt32> iterator(0);
			const ndBrainMatrix& trainingImages = *sourceTrainingImages;
			auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, &trainingImages, trainingLabels, &miniBatchArray, &failCount](ndInt32 threadIndex, ndInt32)
			{
				class CategoricalLoss : public ndBrainLossCategoricalCrossEntropy
				{
					public:
					CategoricalLoss(ndInt32 size, ndUnsigned32* const failCount, ndUnsigned32 entry)
						:ndBrainLossCategoricalCrossEntropy(size)
						,m_entry(entry)
						,m_failCount(failCount)
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
						}

						ndBrainLossCategoricalCrossEntropy::GetLoss(output, loss);
					}

					ndUnsigned32 m_entry;
					ndUnsigned32* m_failCount;
				};

				for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
				{
					ndBrainTrainer& trainer = *m_trainers[i];
					ndUnsigned32 index = miniBatchArray[i];
					CategoricalLoss loss(m_brain.GetOutputSize(), &failCount[threadIndex], index);

					loss.SetTruth((*trainingLabels)[ndInt32(index)]);
					trainer.BackPropagate(trainingImages[ndInt32(index)], loss);
				}
			});

			ndBrain bestBrain(m_brain);
			ndBrainOptimizerAdam optimizer;

			ndInt32 minTestFail = ndInt32(testLabels->GetCount());
			ndInt32 minTrainingFail = ndInt32(trainingLabels->GetCount());
			ndInt32 batches = minTrainingFail / m_miniBatchSize;
			//batches = 1;

			// so far best training result on the cifar-10 data set
			optimizer.SetRegularizer(ndBrainFloat(0.0f));		// with dropout out train score (99.838%)
			//optimizer.SetRegularizer(ndBrainFloat(0.0f));		// base score train score       (99.540%)
			//optimizer.SetRegularizer(ndBrainFloat(1.0e-5f));	// with dropout out train score (99.784%)
			//optimizer.SetRegularizer(ndBrainFloat(1.0e-3f));	// with dropout out train score (99.722%)

			
			ndArray<ndUnsigned32> shuffleBuffer;
			for (ndInt32 i = 0; i < trainingLabels->GetCount(); ++i)
			{
				shuffleBuffer.PushBack(ndUnsigned32(i));
			}

			for (ndInt32 epoch = 0; epoch < 500; ++epoch)
			{
				ndInt32 start = 0;
				ndMemSet(failCount, ndUnsigned32(0), D_MAX_THREADS_COUNT);

				m_brain.EnableDropOut();
				m_brain.UpdateDropOut();
				shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
				for (ndInt32 batch = 0; batch < batches; ++batch)
				{
					iterator = 0;
					ndMemCpy(miniBatchArray, &shuffleBuffer[start], m_miniBatchSize);
					ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
					optimizer.Update(this, m_trainers, m_learnRate);
					start += m_miniBatchSize;
				}
				m_brain.DisableDropOut();

				ndInt32 trainFail = 0;
				for (ndInt32 i = 0; i < GetThreadCount(); ++i)
				{
					trainFail += failCount[i];
				}

				bool test = (trainFail <= minTrainingFail);
				//bool test = (testFail < minTestFail) || ((testFail == minTestFail) && (trainFail < minTrainingFail));
				//bool test = (trainFail < minTrainingFail) || ((trainFail == minTrainingFail) && (testFail < minTestFail));
				if (test)
				{
					auto CrossValidateTest = ndMakeObject::ndFunction([this, &iterator, testLabels, testImages, &failCount](ndInt32 threadIndex, ndInt32)
					{
						ndBrainFloat outputBuffer[32];
						ndBrainMemVector output(outputBuffer, m_brain.GetOutputSize());

						failCount[threadIndex] = 0;
						for (ndInt32 i = iterator++; i < testLabels->GetCount(); i = iterator++)
						{
							const ndBrainVector& truth = (*testLabels)[i];
							const ndBrainVector& input = (*testImages)[i];
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
					//test = testFail <= minTestFail;
					//if (test)
					//{
					//	minTestFail = testFail;
					//}

					minTestFail = testFail;
					minTrainingFail = trainFail;
					bestBrain.CopyFrom(m_brain);
					ndInt32 size = batches * m_miniBatchSize;
					ndExpandTraceMessage("  epoch: %d", epoch);
					ndExpandTraceMessage("  success rate:%f%%", (ndFloat32)(size - minTrainingFail) * 100.0f / (ndFloat32)size);
					ndExpandTraceMessage("  training fail count:%d", minTrainingFail);
					ndExpandTraceMessage("  test fail count:%d\n", minTestFail);
				}
			}
			m_brain.CopyFrom(bestBrain);
#endif
		}

		ndBrain& m_brain;
		ndArray<ndBrainTrainer*> m_trainers;
		ndReal m_learnRate;
		ndInt32 m_miniBatchSize;
	};
	
	if (trainingLabels && trainingImages)
	{
		ndBrain brain;
		ndFixSizeArray<ndBrainLayer*, 32> layers;
		
		ndInt32 height = 32;
		ndInt32 width = trainingImages->GetColumns() / (height * 3);
		ndAssert((3 * height * width) == trainingImages->GetColumns());
	
		const CONVOLUTIONAL_LAYER* conv;
		const ndBrainLayerImagePolling_2x2* pooling;

		// so far the simplest configuration seems to yield better results
		layers.PushBack(new CONVOLUTIONAL_LAYER(width, height, 3, 3, 64));
		conv = (CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
		layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

		layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
		pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

		layers.PushBack(new CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 64));
		conv = (CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
		layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

		layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
		pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

		layers.PushBack(new CONVOLUTIONAL_LAYER(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 64));
		conv = (CONVOLUTIONAL_LAYER*)(layers[layers.GetCount() - 1]);
		layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

		layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
		pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

		layers.PushBack(new LINEAR_LAYER(layers[layers.GetCount() - 1]->GetOutputSize(), LINEAR_LAYER_NEURONS));
		layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

		layers.PushBack(new LINEAR_LAYER(layers[layers.GetCount() - 1]->GetOutputSize(), LINEAR_LAYER_NEURONS));
		layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
		layers.PushBack(new ndBrainLayerActivationCategoricalSoftmax(layers[layers.GetCount() - 1]->GetOutputSize()));
	
		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain.AddLayer(layers[i]);
		}
		brain.InitWeights();
		ndExpandTraceMessage("training cifar-10 database, number of parameters %d\n", brain.GetNumberOfParameters());

		SupervisedTrainer optimizer(&brain);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *trainingImages, *testLabels, *testImages);
		time = ndGetTimeInMicroseconds() - time;
	
		//char path[256];
		//ndGetWorkingFileName("cifar-10-batches-bin/cifar-cnn-dnn", path);
		const ndString path(ndGetWorkingFileName("cifar-10-batches-bin/cifar-cnn-dnn"));
		
		ndBrainSave::Save(&brain, path.GetStr());
		ValidateData("training data", brain, *trainingLabels, *trainingImages, *srcTrainingImages);
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

static void Cifar10TestSet()
{
	ndSharedPtr<ndBrainMatrix> testLabels;
	ndSharedPtr<ndBrainMatrix> testImages;
	ndSharedPtr<ndBrainMatrix> srcTestImages;
	LoadTestData(testImages, srcTestImages, testLabels);
	
	if (testLabels && testImages)
	{
		//char path[256];
		//ndGetWorkingFileName("cifar-10-batches-bin/cifar-cnn-dnn", path);
		const ndString path(ndGetWorkingFileName("cifar-10-batches-bin/cifar-cnn-dnn"));
	
		ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path.GetStr()));
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		ndInt32 numbeOfParam = brain->GetNumberOfParameters();
		ndExpandTraceMessage("cifar-10 database, number of Parameters %d\n", numbeOfParam);
		ValidateData("test data", *(*brain), *testLabels, *testImages, *srcTestImages);
		time = ndGetTimeInMicroseconds() - time;
		ndExpandTraceMessage("time %f (sec)\n\n", ndFloat64(time) / 1000000.0f);
	}
}

void ndCifar10ImageClassification()
{
	ndSetRandSeed(47);

	Cifar10TrainingSet();
	//Cifar10TestSet();
}
