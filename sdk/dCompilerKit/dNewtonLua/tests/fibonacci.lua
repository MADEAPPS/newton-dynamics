
	-- Naive recursive
	function Fibonacci.naive(n)
	  local function inner(m)
		if m < 2 then
		  return m
		end
		return inner(m-1) + inner(m-2)
	  end
	  return inner(n)
	end

	-- Iterative
	function Fibonacci.iterative(n)
	  a, b = 0, 1
	  for i = 1, n do
		a, b = b, a + b
	  end
	  return a
	end