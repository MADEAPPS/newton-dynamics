// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <vector>
#include <random> 

int startingPosition = 1;
int cliffPosition = 0;
int endPosition = 5;

int cliffReward = 0;
int goalStateReward = 5;


static std::mt19937 generator;

float Rand()
{
	unsigned minValue = std::mt19937::min();
	unsigned maxValue = std::mt19937::max();
	unsigned spand(maxValue - minValue);
	float r = float(generator()) / (float)spand;
	return r;
}

int Reward(int currentPosition)
{
	if (currentPosition <= cliffPosition)
	{
		return cliffReward;
	}
	else if (currentPosition >= endPosition)
	{
		return goalStateReward;
	}
	return 0;
}


bool IsTerminating(int currentPosition)
{
	if (currentPosition <= cliffPosition)
		return true;
	if (currentPosition >= endPosition)
		return true;
	return false;
}

int Strategy()
{
	float r = Rand();
	if (r > 0.5f)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

int ActionValueMapping(int x)
{
	 //return 0 if x == -1 else 1
	return (x == -1) ? 0 : 1;
}

int main()
{
	int n_iter = 10;

	generator.seed(42);
	
	struct StatePair
	{
		StatePair()
		{
			m_value[0] = 0;
			m_value[1] = 0;
		}
		int m_value[2];
	};

	std::vector<StatePair> hits;
	std::vector<StatePair> valueSum;
	for (int i = 0; i < endPosition + 1; ++i)
	{
		hits.push_back(StatePair());
		valueSum.push_back(StatePair());
	}

	struct StateAction
	{
		StateAction(int state, int action)
			:m_state(state)
			,m_action(action)
		{
		}
		int m_state;
		int m_action;
	};
	
	for (int i = 0; i < n_iter; ++i)
	{
		std::vector<StateAction> positionHistory;
		
		int currentAction = Strategy();
		int currentPosition = startingPosition;
		while (1)
		{
			//positionHistory.push_back(currentPosition);
			positionHistory.push_back(StateAction(currentPosition, currentAction));
			if (IsTerminating(currentPosition))
			{
				break;
			}

			currentPosition += Strategy();
		}

		int currentReward = Reward(currentPosition);

		for (int j = 0; j < int (positionHistory.size()); j++)
		{
			//int pos = positionHistory[j];
			StateAction stateAction(positionHistory[j]);
			int pos = stateAction.m_state;
			int mapping = ActionValueMapping(stateAction.m_action);
			hits[pos].m_value[mapping] += 1;
			valueSum[pos].m_value[mapping] += currentReward;
		}

		printf("%d average reward: [", i);
		for (int j = 0; j < int (valueSum.size()); j++)
		{
			printf("%f ", float	(valueSum[j].m_value[0])/hits[j].m_value[0]);
		}
		printf(" ; ");
		for (int j = 0; j < int(valueSum.size()); j++)
		{
			printf("%f ", float(valueSum[j].m_value[1]) / hits[j].m_value[1]);
		}
		printf("]\n");
	}

    return 0;
}

