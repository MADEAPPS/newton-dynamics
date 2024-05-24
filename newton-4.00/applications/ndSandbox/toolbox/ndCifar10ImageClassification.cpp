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

static void LoadTrainingData(ndSharedPtr<ndBrainMatrix>& trainingImages, ndSharedPtr<ndBrainMatrix>& trainingLabels)
{
	const ndInt32 batches = 5;
	const ndInt32 pixelSize = 32 * 32;
	char filename[1024];
	char outPathName[1024];
	ndUnsigned8 data[pixelSize * 3];

	//ndInt32 dataAugmentation = 4;
	trainingLabels = new ndBrainMatrix(ndInt32(batches * 10000), ndInt32(10));
	trainingImages = new ndBrainMatrix(ndInt32(batches * 10000), ndInt32(pixelSize * 3));

	ndBrainMatrix& labelMatrix = *(*trainingLabels);
	ndBrainMatrix& imageMatrix = *(*trainingImages);

	ndInt32 base = 0;
	labelMatrix.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < batches; ++i)
	{
		sprintf(filename, "cifar-10-batches-bin/data_batch_%d.bin", i + 1);
		ndGetWorkingFileName(filename, outPathName);
		FILE* const fp = fopen(outPathName, "rb");
		if (fp)
		{
			for (ndInt32 j = 0; j < 10000; ++j)
			{
				size_t ret = 0;
				ndInt32 label = 0;
				ret = fread(&label, 1, 1, fp);

				labelMatrix[j + base][label] = ndBrainFloat(1.0f);
				ndBrainVector& image = imageMatrix[j + base];
				ret = fread(data, 1, pixelSize * 3, fp);
				for (ndInt32 k = 0; k < pixelSize * 3; ++k)
				{
					image[k] = ndBrainFloat(data[k]) / ndBrainFloat(255.0f);
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

#if 0
	dataAugmentation--;
	if (dataAugmentation)
	{
		// flip images
		const ndInt32 size = batches * 10000;
		for (ndInt32 i = 0; i < size; ++i)
		{
			labelMatrix[size + i].Set(labelMatrix[i]);

			ndBrainVector& dstImage = imageMatrix[i + size];
			const ndBrainVector& srcImage = imageMatrix[i];
			for (ndInt32 k = 0; k < 3; ++k)
			{
				ndBrainMemVector dstChannel(&dstImage[k * pixelSize], pixelSize);
				const ndBrainMemVector srcChannel(&srcImage[k * pixelSize], pixelSize);
				for (ndInt32 y = 0; y < 32; ++y)
				{
					ndBrainMemVector dstRow(&dstChannel[y * 32], 32);
					const ndBrainMemVector srcRow(&srcChannel[y * 32], 32);
					for (ndInt32 x = 0; x < 32; ++x)
					{
						dstRow[x] = srcRow[31 - x];
					}
				}
			}
		}
	}

	dataAugmentation-= 2;
	if (dataAugmentation)
	{
		// rotate image by a random angle form 1 to 5 degrees
		const ndInt32 size = batches * 10000 * 2;
		for (ndInt32 i = 0; i < size; ++i)
		{
			labelMatrix[size + i].Set(labelMatrix[i]);

			const ndBrainFloat angle = ndDegreeToRad * (ndRand() * ndBrainFloat(4.0f) + ndBrainFloat(1.0f));
			const ndBrainFloat sinAngle = ndBrainFloat(ndSin(angle));
			const ndBrainFloat cosAngle = ndBrainFloat(ndCos(angle));

			ndBrainVector& dstImage = imageMatrix[i + size];
			const ndBrainVector& srcImage = imageMatrix[i];
			dstImage.Set(srcImage);
			for (ndInt32 k = 0; k < 3; ++k)
			{
				ndBrainMemVector dstChannel(&dstImage[k * pixelSize], pixelSize);
				const ndBrainMemVector srcChannel(&srcImage[k * pixelSize], pixelSize);

				ndBrainFloat inY = ndBrainFloat(-15.5f);
				for (ndInt32 y = 0; y < 32; ++y)
				{
					ndBrainFloat inX = ndBrainFloat(-15.5f);
					ndBrainMemVector dstRow(&dstChannel[y * 32], 32);
					for (ndInt32 x = 0; x < 32; ++x)
					{
						ndInt32 rotX = ndInt32(cosAngle * inX - sinAngle * inY + ndBrainFloat(16.0f));
						ndInt32 rotY = ndInt32(sinAngle * inX + cosAngle * inY + ndBrainFloat(16.0f));
						if ((rotX >= 0) && (rotX < 32) && (rotY >= 0) && (rotY < 32))
						{
							dstRow[x] = srcChannel[rotY * 32 + rotX];
						}
						inX += ndBrainFloat(1.0f);
					}
					inY += ndBrainFloat(1.0f);
				}
			}
		}
	}
#endif
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
			pBits[y][x][0] = unsigned char(src[0 * 32 * 32] * 255.0f);
			pBits[y][x][1] = unsigned char(src[1 * 32 * 32] * 255.0f);
			pBits[y][x][2] = unsigned char(src[2 * 32 * 32] * 255.0f);
			src ++;
		}
	}

	char name1[256];
	sprintf(name1, "%s_%d.png", name, index);
	index++;
	lodepng_encode_file(name1, &pBits[0][0][0], 32, 32, LCT_RGB, 8);
}

static void LoadTestData(ndSharedPtr<ndBrainMatrix>& images, ndSharedPtr<ndBrainMatrix>& srcImages, ndSharedPtr<ndBrainMatrix>& labels)
{
	const ndInt32 pixelSize = 32 * 32;
	char outPathName[1024];
	ndUnsigned8 data[pixelSize * 3];

	labels = new ndBrainMatrix(ndInt32(10000), ndInt32(10));
	images = new ndBrainMatrix(ndInt32(10000), ndInt32(pixelSize * 3));
	srcImages = new ndBrainMatrix(ndInt32(10000), ndInt32(pixelSize * 3));

	ndBrainMatrix& labelMatrix = *(*labels);
	ndBrainMatrix& imageMatrix = *(*images);
	ndBrainMatrix& srcImageMatrix = *(*srcImages);

	labelMatrix.Set(ndBrainFloat(0.0f));
	ndGetWorkingFileName("cifar-10-batches-bin/test_batch.bin", outPathName);
	FILE* const fp = fopen(outPathName, "rb");
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

	brain.DisableDropOut();

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
	#define BASH_BUFFER_SIZE	64

	ndSharedPtr<ndBrainMatrix> testLabels;
	ndSharedPtr<ndBrainMatrix> testImages;
	ndSharedPtr<ndBrainMatrix> srcTestImages;
	ndSharedPtr<ndBrainMatrix> trainingLabels;
	ndSharedPtr<ndBrainMatrix> trainingImages;

	LoadTestData(testImages, srcTestImages, testLabels);
	LoadTrainingData(trainingImages, trainingLabels);

	class SupervisedTrainer : public ndBrainThreadPool
	{
		public:
		SupervisedTrainer(ndBrain* const brain)
			:ndBrainThreadPool()
			,m_brain(*brain)
			,m_learnRate(ndReal(5.0e-4f))
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
		}

		~SupervisedTrainer()
		{
			for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
			{
				delete m_trainers[i];
			}
		}

		void GenerateAugmentedImageBash(const ndBrainMatrix& src, ndBrainMatrix& dst) const
		{
			const ndInt32 pixelSize = 32 * 32;
			ndBrainFixSizeVector<pixelSize * 3> tmpImage;
			ndBrainFixSizeVector<pixelSize * 3> rotImage;
			for (ndInt32 i = 0; i < src.GetCount(); ++i)
			{
				if (ndRandInt() & 1)
				{
					tmpImage.Set(src[i]);
				}
				else
				{
					const ndBrainVector& srcImage = src[i];
					for (ndInt32 k = 0; k < 3; ++k)
					{
						ndBrainMemVector dstChannel(&tmpImage[k * pixelSize], pixelSize);
						const ndBrainMemVector srcChannel(&srcImage[k * pixelSize], pixelSize);
						for (ndInt32 y = 0; y < 32; ++y)
						{
							ndBrainMemVector dstRow(&dstChannel[y * 32], 32);
							const ndBrainMemVector srcRow(&srcChannel[y * 32], 32);
							for (ndInt32 x = 0; x < 32; ++x)
							{
								dstRow[x] = srcRow[31 - x];
							}
						}
					}
				}

				rotImage.Set(tmpImage);
				
				const ndBrainFloat angle = ndBrainFloat(ndDegreeToRad * (ndRand() * ndBrainFloat(10.0f) - ndBrainFloat(5.0f)));
				const ndBrainFloat sinAngle = ndBrainFloat(ndSin(angle));
				const ndBrainFloat cosAngle = ndBrainFloat(ndCos(angle));

				for (ndInt32 k = 0; k < 3; ++k)
				{
					ndBrainMemVector dstChannel(&rotImage[k * pixelSize], pixelSize);
					const ndBrainMemVector srcChannel(&tmpImage[k * pixelSize], pixelSize);

					ndBrainFloat inY = ndBrainFloat(-15.5f);
					for (ndInt32 y = 0; y < 32; ++y)
					{
						ndBrainFloat inX = ndBrainFloat(-15.5f);
						ndBrainMemVector dstRow(&dstChannel[y * 32], 32);
						for (ndInt32 x = 0; x < 32; ++x)
						{
							ndInt32 rotX = ndInt32(cosAngle * inX - sinAngle * inY + ndBrainFloat(16.0f));
							ndInt32 rotY = ndInt32(sinAngle * inX + cosAngle * inY + ndBrainFloat(16.0f));
							if ((rotX >= 0) && (rotX < 32) && (rotY >= 0) && (rotY < 32))
							{
								dstRow[x] = srcChannel[rotY * 32 + rotX];
							}
							inX += ndBrainFloat(1.0f);
						}
						inY += ndBrainFloat(1.0f);
					}
				}

				dst[i].Set(rotImage);
			}
		}

		void Optimize(ndBrainMatrix* const trainingLabels, const ndBrainMatrix* const sourceTrainingImages,
					  ndBrainMatrix* const testLabels, ndBrainMatrix* const testDigits)
		{
			ndUnsigned32 miniBashArray[64];
			ndUnsigned32 failCount[D_MAX_THREADS_COUNT];

			ndAtomic<ndInt32> iterator(0);
			ndBrainMatrix trainingImages(sourceTrainingImages->GetRows(), sourceTrainingImages->GetColumns());
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, &trainingImages, trainingLabels, &miniBashArray, &failCount](ndInt32 threadIndex, ndInt32)
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

				//const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
				//for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					ndBrainTrainer& trainer = *m_trainers[i];
					ndUnsigned32 index = miniBashArray[i];
					CategoricalLoss loss(m_brain.GetOutputSize(), &failCount[threadIndex], index);

					loss.SetTruth((*trainingLabels)[ndInt32(index)]);
					trainer.BackPropagate(trainingImages[ndInt32(index)], loss);
				}
			});

			ndBrain bestBrain(m_brain);
			ndBrainOptimizerAdam optimizer;

			ndInt32 minTestFail = testLabels->GetCount();
			ndInt32 batches = trainingLabels->GetCount() / m_bashBufferSize;
			//batches = 1;

			// so far best training result on the cifar-10 data set
			optimizer.SetRegularizer(ndBrainFloat(0.0e-5f));	// test data score (83.96%)
			//optimizer.SetRegularizer(ndBrainFloat(1.0e-4f));	// test data score (83.10%)
			//optimizer.SetRegularizer(ndBrainFloat(1.0e-3f));	// test data score (83.22%)
			//optimizer.SetRegularizer(ndBrainFloat(3.0e-5f));	// test data score (%)
			
			ndArray<ndUnsigned32> shuffleBuffer;
			for (ndInt32 i = 0; i < trainingLabels->GetCount(); ++i)
			{
				shuffleBuffer.PushBack(ndUnsigned32(i));
			}

			for (ndInt32 epoch = 0; epoch < 5000; ++epoch)
			{
				ndInt32 start = 0;
				ndMemSet(failCount, ndUnsigned32(0), D_MAX_THREADS_COUNT);

				GenerateAugmentedImageBash(*sourceTrainingImages, trainingImages);

				m_brain.EnableDropOut();
				for (ndInt32 bash = 0; bash < batches; ++bash)
				{
					ndMemCpy(miniBashArray, &shuffleBuffer[start], m_bashBufferSize);

					iterator = 0;
					ndBrainThreadPool::ParallelExecute(BackPropagateBash);
					optimizer.Update(this, m_trainers, m_learnRate);

					m_brain.UpdateDropOut();
					start += m_bashBufferSize;
				}

				ndInt32 trainFailed = 0;
				for (ndInt32 i = 0; i < GetThreadCount(); ++i)
				{
					trainFailed += failCount[i];
				}

				auto CrossValidateTest = ndMakeObject::ndFunction([this, &iterator, testDigits, testLabels, &failCount](ndInt32 threadIndex, ndInt32)
				{
					ndBrainFloat outputBuffer[32];
					ndBrainMemVector output(outputBuffer, m_brain.GetOutputSize());

					failCount[threadIndex] = 0;
					//const ndStartEnd startEnd(testLabels->GetCount(), threadIndex, threadCount);
					//for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
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
				m_brain.DisableDropOut();
				ndBrainThreadPool::ParallelExecute(CrossValidateTest);

				ndInt32 testFail = 0;
				for (ndInt32 j = 0; j < GetThreadCount(); ++j)
				{
					testFail += failCount[j];
				}

				if (testFail <= minTestFail)
				{
					minTestFail = testFail;
					bestBrain.CopyFrom(m_brain);
					ndInt32 size = batches * m_bashBufferSize;
					ndExpandTraceMessage("success rate: %f%%   ", (ndFloat32)(size - trainFailed) * 100.0f / (ndFloat32)size);
					ndExpandTraceMessage("epoch: %d  ", epoch);
					ndExpandTraceMessage("train failed: %d   ", trainFailed);
					ndExpandTraceMessage("test failed: %d\n", minTestFail);
				}

				shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
			}
			m_brain.CopyFrom(bestBrain);
		}

		ndBrain& m_brain;
		ndArray<ndBrainTrainer*> m_trainers;
		ndReal m_learnRate;
		ndInt32 m_bashBufferSize;
	};
	
	if (trainingLabels && trainingImages)
	{
		ndBrain brain;
		ndFixSizeArray<ndBrainLayer*, 32> layers;
		
		ndInt32 height = 32;
		ndInt32 width = trainingImages->GetColumns() / (height * 3);
		ndAssert((3 * height * width) == trainingImages->GetColumns());
	
		const ndBrainLayerImagePolling_2x2* pooling;
		const ndBrainLayerConvolutionalWithDropOut_2d* conv;
	
		#if 0
			#define ACTIVATION_TYPE	ndBrainLayerReluActivation
		#else
			#define ACTIVATION_TYPE	ndBrainLayerTanhActivation
		#endif
	
		#define ND_CNN_MODEL 0

		#if ND_CNN_MODEL == 0
			// so far the simplest configuration seems to yield better results
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(width, height, 3, 3, 128));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(pooling->GetOutputSize()));

			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 128));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(pooling->GetOutputSize()));

			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 64));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(pooling->GetOutputSize()));

			ndInt32 neuronsPerLayers = 64;
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
			layers.PushBack(new ndBrainLayerCategoricalSoftmaxActivation(layers[layers.GetCount() - 1]->GetOutputSize()));

		#elif ND_CNN_MODEL == 1

			// trying more layer and with more filters, four time slower and not better results
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(width, height, 3, 3, 32));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels(), 3, 32));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
	
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 64));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels(), 3, 64));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);
	
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 128, 0.7f));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels(), 3, 128, 0.7f));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			//ndInt32 neuronsPerLayers = 64;
			//layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
			//layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			//layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
			//layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
			layers.PushBack(new ndBrainLayerCategoricalSoftmaxActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
			
		#else
		
			// trying VGG16 style network
			const ndBrainLayerImagePadding* paddLayer;
			layers.PushBack(new ndBrainLayerImagePadding(width, height, 3, 5));
			paddLayer = (ndBrainLayerImagePadding*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(paddLayer->GetOutputWidth(), paddLayer->GetOutputHeight(), paddLayer->GetOutputChannels(), paddLayer->GetFilterSize(), 16));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels(), conv->GetFilterSize(), 32));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			//layers.PushBack(new ndBrainLayerImagePadding(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3));
			//paddLayer = (ndBrainLayerImagePadding*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), paddLayer->GetFilterSize(), 32));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));
			layers.PushBack(new ndBrainLayerImagePolling_2x2(conv->GetOutputWidth(), conv->GetOutputHeight(), conv->GetOutputChannels()));
			pooling = (ndBrainLayerImagePolling_2x2*)(layers[layers.GetCount() - 1]);

			layers.PushBack(new ndBrainLayerConvolutionalWithDropOut_2d(pooling->GetOutputWidth(), pooling->GetOutputHeight(), pooling->GetOutputChannels(), 3, 32));
			conv = (ndBrainLayerConvolutionalWithDropOut_2d*)(layers[layers.GetCount() - 1]);
			layers.PushBack(new ACTIVATION_TYPE(conv->GetOutputSize()));

			ndInt32 neuronsPerLayers = 64;
			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new ndBrainLayerLinearWithDropOut(layers[layers.GetCount() - 1]->GetOutputSize(), neuronsPerLayers));
			layers.PushBack(new ACTIVATION_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));

			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), trainingLabels->GetColumns()));
			layers.PushBack(new ndBrainLayerCategoricalSoftmaxActivation(layers[layers.GetCount() - 1]->GetOutputSize()));

		#endif
	
		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			brain.AddLayer(layers[i]);
		}
		brain.InitWeightsXavierMethod();
		ndExpandTraceMessage("training cifar-10 database, number of parameters %d\n", brain.GetNumberOfParameters());

		SupervisedTrainer optimizer(&brain);
		ndUnsigned64 time = ndGetTimeInMicroseconds();
		optimizer.Optimize(*trainingLabels, *trainingImages, *testLabels, *testImages);
		time = ndGetTimeInMicroseconds() - time;
	
		char path[256];
		ndGetWorkingFileName("cifar-10-batches-bin/cifar-cnn-dnn", path);
		
		ndBrainSave::Save(&brain, path);
		ValidateData("training data", brain, *trainingLabels, *trainingImages, *srcTestImages);
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
		char path[256];
		ndGetWorkingFileName("cifar-10-batches-bin/cifar-cnn-dnn", path);
	
		ndSharedPtr<ndBrain> brain (ndBrainLoad::Load(path));
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
	ndSetRandSeed(12345);

	Cifar10TrainingSet();
	//Cifar10TestSet();
}
