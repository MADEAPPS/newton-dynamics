#include <stdlib.h>
#include <map>
#include <vector>
#include <random> 

static std::mt19937 generator;

int Rand()
{
	unsigned minValue = std::mt19937::min();
	unsigned maxValue = std::mt19937::max();
	int r = generator() % 4;
	return r;
}

class Point
{
	public:
	Point()
		:m_x(0), m_y(0)
	{
	}

	Point(int x, int y)
		:m_x(x), m_y(y)
	{
	}

	bool operator==(const Point& src) const
	{
		return (m_x == src.m_x) && (m_y == src.m_y);
	}

	int m_x;
	int m_y;
};

class Direction
{
	public:
	enum Dir
	{
		north,
		south,
		east,
		west
	};

	Direction()
	{
		m_dir[0] = north;
		m_dir[1] = south;
		m_dir[2] = east;
		m_dir[3] = west;
	}

	Dir m_dir[4];
};

class SimpleGridWorld
{
	public:
	struct StateStep
	{
		Point m_posit;
		int m_action;
		int m_reward;
		bool m_isTerminal;
	};

	SimpleGridWorld(int width = 5, int height = 5)
		:m_width(width) 
		,m_height(height)
		,m_debug(false)
		,m_actionSpace()
		,m_goal(m_width - 1, 0)
		,m_curPosition(0, m_height - 1)
	{
		reset();
	}

	void reset()
	{
		m_goal = Point(m_width - 1, 0);
		m_curPosition = Point(0, m_height - 1);
	}

	StateStep Step(int action)
	{
		switch (action)
		{
			case Direction::north:
				m_curPosition = Point(m_curPosition.m_x, (m_curPosition.m_y + 1) < m_height ? m_curPosition.m_y + 1 : m_curPosition.m_y);
				break;

			case Direction::south:
				m_curPosition = Point(m_curPosition.m_x, (m_curPosition.m_y - 1) >= 0 ? m_curPosition.m_y - 1 : m_curPosition.m_y);
				break;

			case Direction::east:
				m_curPosition = Point((m_curPosition.m_x + 1) < m_width ? m_curPosition.m_x + 1 : 0, m_curPosition.m_y);
				break;

			case Direction::west:
				m_curPosition = Point((m_curPosition.m_x - 1) >= 0 ? m_curPosition.m_x - 1 : 0, m_curPosition.m_y);
				break;
		}
		
		StateStep state;
		state.m_reward = -1;
		state.m_action = action;
		state.m_posit = m_curPosition;
		state.m_isTerminal = (m_curPosition == m_goal);
		return state;
	}

	void Print()
	{
		if (m_debug)
		{
			std::string res;
			for (int y = m_height - 1; y >= 0; --y)
			{
				for (int x = 0; x < m_width; ++x)
				{
					if ((m_goal.m_x == x) && (m_goal.m_y == y))
					{
						if ((m_curPosition.m_x == x) && (m_curPosition.m_y == y))
						{
							res += "@";
						}
						else
						{
							res += "o";
						}
						continue;
					}
					if ((m_curPosition.m_x == x) && (m_curPosition.m_y == y))
					{
						res += "x";
					}
					else
					{
						res += "-";
					}
				}
				res += "\n";
			}
			printf("%s\n", res.c_str());
		}
	}

	int m_width;
	int m_height;
	bool m_debug;
	Point m_goal;
	Point m_curPosition;
	Direction m_actionSpace;
};

class MonteCarloGeneration
{
	public:
	MonteCarloGeneration()
		:m_enviroment(5, 5)
		,m_maxSteps(10)
	{
	}

	int Choice()
	{
		int e = Rand();
		return m_enviroment.m_actionSpace.m_dir[e];
	}

	void Run(std::vector<SimpleGridWorld::StateStep>& buffer)
	{
		int steps = 0;
		bool terminal = false;
		m_enviroment.reset();
		m_enviroment.Print();
		Point state = m_enviroment.m_curPosition;

static int xxxx;
xxxx++;

		buffer.resize(0);
		while (!terminal)
		{
			int action = Choice();
			SimpleGridWorld::StateStep nextState (m_enviroment.Step(action));
			m_enviroment.Print();

			SimpleGridWorld::StateStep bufferState;
			bufferState.m_posit = state;
			bufferState.m_action = action;
			bufferState.m_reward = nextState.m_reward;
			terminal = nextState.m_isTerminal;
			buffer.push_back(bufferState);
			state = nextState.m_posit;
			steps++;
			if (steps >= m_maxSteps)
			{
				//printf("Terminate early due to large number of steps\n");
				terminal = true;
			}
		}
	}

	int CalculateTotalReward(std::vector<SimpleGridWorld::StateStep>& buffer)
	{
		int reward = 0;
		for (int i = 0; i < int (buffer.size()); ++i)
		{
			reward += buffer[i].m_reward;
		}
		return reward;
	}

	SimpleGridWorld m_enviroment;
	int m_maxSteps;
};

class MonteCarloExperiment
{
	public:
	class DictKey
	{
		public:
		DictKey(Point& state, int action)
			:m_state(state)
			,m_action(action)
		{
		}

		bool operator<(const DictKey& src) const
		{
			int code0 = (((    m_state.m_x << 8) +     m_state.m_y) << 8) +     m_action;
			int code1 = (((src.m_state.m_x << 8) + src.m_state.m_y) << 8) + src.m_action;
			return code0 < code1;
		}

		Point m_state;
		int m_action;
	};

	class DefualtDict : public std::map<DictKey, float>
	{
		public:
		DefualtDict()
			:std::map<DictKey, float>()
		{
		}

		float GetValue(const DictKey& key) const
		{
			float value = 0;
			const_iterator node = find(key);
			if (node != end())
			{
				value = node->second;
			}
			return value;
		}
	};

	MonteCarloExperiment()
		:m_generator()
	{
	}

	float ActionValue(Point& state, int action)
	{
		DictKey key(state, action);
		float value = 0.0f;
		if (m_counts.find(key) != m_counts.end())
		{
			float num = m_values.GetValue(key);
			float den = m_counts.GetValue(key);
			value = num/den;
		}
		return value;
	}

	void RunEpisode()
	{
		std::vector<SimpleGridWorld::StateStep> trajectoty;
		m_generator.Run(trajectoty);
		int episodeReward = 0;
		//m_values.clear();
		//m_counts.clear();
		for (int i = trajectoty.size() - 1; i >= 0; --i)
		{
			SimpleGridWorld::StateStep stateActionReward(trajectoty[i]);
			DictKey key(stateActionReward.m_posit, stateActionReward.m_action);
			episodeReward += stateActionReward.m_reward;
			m_values[key] += episodeReward;
			m_counts[key] += 1.0f;
		}
	}

	MonteCarloGeneration m_generator;
	DefualtDict m_values;
	DefualtDict m_counts;
};

void TestEnviroment()
{
	printf("This shows a simple visualization of the environment\n");
	SimpleGridWorld s(5, 5); s.Print();

	s.Step(Direction::south); s.Print();
	s.Step(Direction::south); s.Print();
	s.Step(Direction::south); s.Print();
	s.Step(Direction::south); s.Print();

	s.Step(Direction::east); s.Print();
	s.Step(Direction::east); s.Print();
	s.Step(Direction::east); s.Print();
	s.Step(Direction::east); s.Print();
}

void TestTrajectory()
{
	MonteCarloGeneration generator;
	std::vector<SimpleGridWorld::StateStep> buffer;
	generator.Run(buffer);
	printf("total reward: %d", generator.CalculateTotalReward(buffer));
}

void RunEpisode()
{
	MonteCarloExperiment agent;
	for (int i = 0; i < 10; ++i)
	{
		agent.RunEpisode();

		printf("Run %d: [", i);
		for (int j = 0; j < sizeof(Direction::m_dir) / sizeof(Direction::m_dir[0]); ++j)
		{
			int d = agent.m_generator.m_enviroment.m_actionSpace.m_dir[j];
			float value = agent.ActionValue(Point(1, 3), d);
			printf("%f ", value);
		}
		printf("]\n");
	}
}

int main()
{
	generator.seed(42);
	//TestEnviroment();
	//TestTrajectory();
	RunEpisode();

    return 0;
}

