#include "stdafx.h"
#include "dProfilerTrace.h"
#include "dTimeTrackerViewer.h"

class dProfilerTrace::dThreadTrace : public dArray<dTimeTrackerRecord>
{
	public:
	dThreadTrace()
		:dArray<dTimeTrackerRecord>()
	{
	}

	~dThreadTrace()
	{
	}

	void AddTrace(Bytef* const compressedData, int compressesDataSize)
	{
		dThreadTrace& me = *this;

		me[me.GetSize() + (1 << DG_TIME_TRACKER_ENTRIES_POWER) - 1].m_start = 0;

		dTimeTrackerRecord* const buffer = &me[me.GetSize()];

		uLongf destLen;
		int compressError = uncompress((Bytef*)buffer, &destLen, compressedData, compressesDataSize);
		dAssert(compressError == Z_OK);
		m_count += 1 << DG_TIME_TRACKER_ENTRIES_POWER;
	}
};

class dProfilerTrace::dTrackerSample
{
	public:
	dTrackerSample(unsigned name, unsigned start, unsigned duration)
		:m_name(name)
		,m_start(start)
		,m_duration(duration)
		,m_children(dArray<dTrackerSample*>())
	{
	}

	~dTrackerSample()
	{
		for (int i = 0; i < m_children.GetSize(); i++) {
			delete m_children[i];
		}
	}

	const void Render(dTimeTrackerViewer* const viewer) const
	{

	}

	unsigned m_name;
	unsigned m_start;
	unsigned m_duration;
	dArray<dTrackerSample*> m_children;
};

class dProfilerTrace::dTrackerThread
{
	public:
	dTrackerThread(unsigned threadName, const dThreadTrace& track)
		:m_frames()
		,m_name(threadName)
		,m_levels_deep(1)
		,m_isOpen(true)
	{
		int index = 0;
		const int maxSize = track.GetSize();
		do {
			dTrackerSample* const trace = GetSample(track, index);

			bool isRootTrace = true;
			const int framesCount = m_frames.GetSize();

			if (framesCount) {
				int x0 = trace->m_start;
				int y0 = m_frames[framesCount - 1]->m_start;
				int y1 = y0 + m_frames[framesCount - 1]->m_duration;
				if (x0 >= y1) {
					m_frames.Push(trace);
				} else {
					index *= 1;
					//assert (0);
				}
			} else {
				m_frames.Push(trace);
			}
		} while (index < maxSize);
	}

	~dTrackerThread()
	{
		for (int i = 0; i < m_frames.GetSize(); i++) {
			delete m_frames[i];
		}
	}

	dTrackerSample* GetSample (const dThreadTrace& track, int& index)
	{
		const dTimeTrackerRecord& record = track[index];
		dTrackerSample* const trace = new dTrackerSample(record.m_nameHash, record.m_start, record.m_duration);
		index ++;
		return trace;
	}

	void dProfilerTrace::dTrackerThread::Render(dTimeTrackerViewer* const viewer)
	{
		dProfilerTrace& root = *viewer->GetTrace();
		const char* const threadName = root.m_nameList[m_name].m_string;

		if (ImGui::CollapsingHeader(threadName, &m_isOpen)) {

			// calculate view area rectangle
			ImVec2 cursorPosit0(ImGui::GetCursorScreenPos());
			for (int i = 0; i < m_levels_deep + 1; i++) {
				ImGui::Text("");
			}
			ImVec2 cursorPosit1(ImGui::GetCursorScreenPos());
			cursorPosit1.x += ImGui::GetWindowSize().x;

			if (ImGui::IsMouseHoveringRect(cursorPosit0, cursorPosit1, false)) {
				// if mouse is over this area save the the rectangle
				root.m_rootNode.m_mouseBoxp0 = cursorPosit0;
				root.m_rootNode.m_mouseBoxp1 = cursorPosit1;
			}

			const dTraceCapture& capture = viewer->GetTrace()->m_rootNode; 

			ImVec2 text_size = ImGui::CalcTextSize("");

			float textPadd = 2.0f;
			float textWitdh = text_size.y;

			float grouping = 4.0f;
			float origin = capture.m_origin;
			float p0 = capture.m_minTime;
			float width = capture.m_windowSize;
			float scale = capture.m_windowSize * capture.m_scale / (capture.m_maxTime - capture.m_minTime);

			ImVec2 box0(0.0f, cursorPosit0.y);
			ImVec2 box1(0.0f, cursorPosit0.y + textWitdh + textPadd * 2.0f);
			
			ImU32 rectColor = 0xff00c000;
			ImU32 groupColor = 0xffa00000;
			ImU32 textColor = 0xff000000;
			//ImU32 borderColor = 0xff00ff00;

			ImDrawList* const draw = ImGui::GetWindowDrawList();

			for (int i = 0; i < m_frames.GetSize(); i ++) {
				const dTrackerSample* const sample0 = m_frames[i];

				float x0 = origin + scale * (sample0->m_start - p0);

				if (x0 >= width) {
					break;
				}

				float x1 = origin + scale * (sample0->m_start + sample0->m_duration - p0);

				if (x1 >= 0.0f) {
					ImU32 color = rectColor;
					while (((x1 - x0) < grouping) && (i < m_frames.GetSize())) {
						const dTrackerSample* const sample1 = m_frames[i + 1];
						float z0 = origin + scale * (sample1->m_start - p0);
						float z1 = origin + scale * (sample1->m_start + sample1->m_duration - p0);
						color = groupColor;
						if ((z1 - z0) >= grouping) {
							break;
						}
						i ++;
					} 

					box0.x = x0;
					box1.x = x1;
					draw->AddRectFilled(box0, box1, color);

					if (sample0 == m_frames[i]) {
						char functionName[256];
						//const char* const functionName = root.m_nameList[sample0->m_name].m_string;
						sprintf (functionName, "%s %d", root.m_nameList[sample0->m_name].m_string, sample0->m_duration);
						ImVec2 text_size = ImGui::CalcTextSize(functionName);
						int t1 = strlen (functionName);
						const char* functionNameEnd = functionName + t1;
						if (text_size.x > (x1 - x0)) {
							int t0 = 0;
							while ((t1 - t0) >= 5) {
								int t = (t0 + t1) / 2;
								text_size = ImGui::CalcTextSize(functionName, functionName + t);
								if (text_size.x > (x1 - x0)) {
									t1 = t;
								} else {
									t0 = t;
								}
							}
							functionNameEnd = functionName + t0;
							text_size = ImGui::CalcTextSize(functionName, functionName + t0);
						}

						if (text_size.x <= (x1 - x0)) {
							ImVec2 textPost (box0);
							textPost.y += textPadd;
							textPost.x += (x1 - x0 - text_size.x) * 0.5f;
							draw->AddText(textPost, textColor, functionName, functionNameEnd);
						}

						sample0->Render(viewer);
					}
				}
			}
		}
	}

	dArray<dTrackerSample*> m_frames;
	int m_name;
	int m_levels_deep;
	bool m_isOpen;
};

class dProfilerTrace::dDataBase
{
	public: 
	dDataBase(FILE* const file)
		:m_file(file)
		,m_trace()
		,m_dictionary()
	{
	}

	public: ~dDataBase()
	{
	}

	int ReadInt()
	{
		int code;
		fread (&code, sizeof (int), 1, m_file);
		return code;
	}

	dTrackerChunkType ReadChunkType()
	{
		return dTrackerChunkType(ReadInt());
	}

	int ReadName(char* const name)
	{
		unsigned size = ReadInt();
		fread(name, size, 1, m_file);
		name[size] = 0;
		return size;
	}

	void ReadCompressedTrack(char* const buffer, int size)
	{
		fread (buffer, size, 1, m_file);
	}

	FILE* m_file;
	dTimeTrackerMap<dThreadTrace, unsigned> m_trace;	
	dTimeTrackerMap<dTrackerString, unsigned> m_dictionary;
};

dProfilerTrace::dProfilerTrace(FILE* const file)
	:m_rootNode()
	,m_nameList()
{
	dDataBase database(file); 

	for (dTrackerChunkType chunkType = database.ReadChunkType(); chunkType != m_traceEnd; chunkType = database.ReadChunkType()) {

		switch (chunkType) 
		{
			case m_traceSamples:
			{
				ReadTrack(database);
				break;
			}

			case m_traceLabel:
			{
				ReadLabels(database);
				break;
			}

			default:
				dAssert(0);
				break;
		}
	}

	dTimeTrackerMap<int, unsigned> nameMap;
	dTimeTrackerMap<dTrackerString, unsigned>::Iterator iter (database.m_dictionary);
	for (iter.Begin(); iter; iter ++) {
		dTrackerString name (iter.GetNode()->GetInfo());
		unsigned key = iter.GetKey();
		nameMap.Insert(m_nameList.GetSize(), key);
		_strrev (name.m_string);
		char* const ptr = strstr (name.m_string, "::");
		if (ptr) {
			*ptr = 0;
		}
		_strrev (name.m_string);
		m_nameList.Push (name);
	}

	dTimeTrackerMap<dThreadTrace, unsigned>::Iterator traceIter (database.m_trace);
	unsigned maxTime = 0;
	unsigned minTime = -1;
	for (traceIter.Begin(); traceIter; traceIter ++) {
		dThreadTrace& track = traceIter.GetNode()->GetInfo();
		const int threadsCount = track.GetSize();
		for (int i = 0; i < threadsCount; i ++) {
			int remapHashIndex = nameMap.Find(track[i].m_nameHash)->GetInfo();
			track[i].m_nameHash = remapHashIndex;
		}
		int treadHashIndex = nameMap.Find(traceIter.GetKey())->GetInfo();

		dTrackerThread* const thread = new dTrackerThread(treadHashIndex, track);
		m_rootNode.m_treads.Push(thread);

		const dArray<dTrackerSample*>& frames = thread->m_frames;
		minTime = dMin(minTime, frames[0]->m_start);
		maxTime = dMax(maxTime, frames[frames.GetSize() - 1]->m_start);
	}
	m_rootNode.m_minTime = float (minTime);
	m_rootNode.m_maxTime = float (maxTime);

	m_rootNode.m_scale = 500.0f;
	m_rootNode.m_origin = m_rootNode.m_minTime;

	for (int i = 1; i < m_rootNode.m_treads.GetSize(); i ++) {
		dTrackerThread* const tmp = m_rootNode.m_treads[i];
		const dTrackerString& name0 = m_nameList[tmp->m_name];
		int index = i;
		for (; index > 0; index --) {
			dTrackerThread* const tread = m_rootNode.m_treads[index - 1];
			const dTrackerString& name1 = m_nameList[tread->m_name];
			int test = strcmp (name0.m_string, name1.m_string);
			if (test <= 0) {
				break;
			}
			m_rootNode.m_treads[index] = tread;
		}
		m_rootNode.m_treads[index] = tmp;
	}
}

dProfilerTrace::~dProfilerTrace()
{
}

void dProfilerTrace::ReadTrack(dDataBase& database)
{
	unsigned nameCRC = database.ReadInt();
	unsigned compressesDataSize = database.ReadInt();

	char* compressedData = dAlloca (char, compressesDataSize + 1024);
	database.ReadCompressedTrack(compressedData, compressesDataSize);

	dTimeTrackerMap<dThreadTrace, unsigned>::dTreeNode* threadNode = database.m_trace.Find(nameCRC);
	if (!threadNode) {
		threadNode = database.m_trace.Insert(nameCRC);
	}
	dThreadTrace& track = threadNode->GetInfo();
	track.AddTrace((Bytef*)compressedData, compressesDataSize);
}

void dProfilerTrace::ReadLabels(dDataBase& database)
{
	for (dTrackerChunkType chunkType = database.ReadChunkType(); chunkType == m_traceLabel; chunkType = database.ReadChunkType()) {
		char name[1024];
		unsigned key = database.ReadInt();
		unsigned size = database.ReadName(name);
		database.m_dictionary.Insert(name, key);
	}
}


//#define IM_ARRAYSIZE(_ARR)      ((int)(sizeof(_ARR)/sizeof(*_ARR)))

void dProfilerTrace::Render (dTimeTrackerViewer* const viewer)
{
/*
    if (ImGui::CollapsingHeader("Layout"))
    {
        if (ImGui::TreeNode("Child regions"))
        {
            ImGui::Text("Without border");
            static int line = 50;
            bool goto_line = ImGui::Button("Goto");
            ImGui::SameLine();
            ImGui::PushItemWidth(100);
            goto_line |= ImGui::InputInt("##Line", &line, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::PopItemWidth();
            ImGui::BeginChild("Sub1", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f,300), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (int i = 0; i < 100; i++)
            {
                ImGui::Text("%04d: scrollable region", i);
                if (goto_line && line == i)
                    ImGui::SetScrollHere();
            }
            if (goto_line && line >= 100)
                ImGui::SetScrollHere();
            ImGui::EndChild();

            ImGui::SameLine();

            ImGui::PushStyleVar(ImGuiStyleVar_ChildWindowRounding, 5.0f);
            ImGui::BeginChild("Sub2", ImVec2(0,300), true);
            ImGui::Text("With border");
            ImGui::Columns(2);
            for (int i = 0; i < 100; i++)
            {
                if (i == 50)
                    ImGui::NextColumn();
                char buf[32];
                sprintf(buf, "%08x", i*5731);
                ImGui::Button(buf, ImVec2(-1.0f, 0.0f));
            }
            ImGui::EndChild();
            ImGui::PopStyleVar();

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Widgets Width"))
        {
            static float f = 0.0f;
            ImGui::Text("PushItemWidth(100)");
            ImGui::SameLine(); ShowHelpMarker("Fixed width.");
            ImGui::PushItemWidth(100);
            ImGui::DragFloat("float##1", &f);
            ImGui::PopItemWidth();

            ImGui::Text("PushItemWidth(GetWindowWidth() * 0.5f)");
            ImGui::SameLine(); ShowHelpMarker("Half of window width.");
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
            ImGui::DragFloat("float##2", &f);
            ImGui::PopItemWidth();

            ImGui::Text("PushItemWidth(GetContentRegionAvailWidth() * 0.5f)");
            ImGui::SameLine(); ShowHelpMarker("Half of available width.\n(~ right-cursor_pos)\n(works within a column set)");
            ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth() * 0.5f);
            ImGui::DragFloat("float##3", &f);
            ImGui::PopItemWidth();

            ImGui::Text("PushItemWidth(-100)");
            ImGui::SameLine(); ShowHelpMarker("Align to right edge minus 100");
            ImGui::PushItemWidth(-100);
            ImGui::DragFloat("float##4", &f);
            ImGui::PopItemWidth();

            ImGui::Text("PushItemWidth(-1)");
            ImGui::SameLine(); ShowHelpMarker("Align to right edge");
            ImGui::PushItemWidth(-1);
            ImGui::DragFloat("float##5", &f);
            ImGui::PopItemWidth();

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Basic Horizontal Layout"))
        {
            ImGui::TextWrapped("(Use ImGui::SameLine() to keep adding items to the right of the preceding item)");

            // Text
            ImGui::Text("Two items: Hello"); ImGui::SameLine();
            ImGui::TextColored(ImVec4(1,1,0,1), "Sailor");

            // Adjust spacing
            ImGui::Text("More spacing: Hello"); ImGui::SameLine(0, 20);
            ImGui::TextColored(ImVec4(1,1,0,1), "Sailor");

            // Button
            ImGui::AlignFirstTextHeightToWidgets();
            ImGui::Text("Normal buttons"); ImGui::SameLine();
            ImGui::Button("Banana"); ImGui::SameLine();
            ImGui::Button("Apple"); ImGui::SameLine();
            ImGui::Button("Corniflower");

            // Button
            ImGui::Text("Small buttons"); ImGui::SameLine();
            ImGui::SmallButton("Like this one"); ImGui::SameLine();
            ImGui::Text("can fit within a text block.");

            // Aligned to arbitrary position. Easy/cheap column.
            ImGui::Text("Aligned");
            ImGui::SameLine(150); ImGui::Text("x=150");
            ImGui::SameLine(300); ImGui::Text("x=300");
            ImGui::Text("Aligned");
            ImGui::SameLine(150); ImGui::SmallButton("x=150");
            ImGui::SameLine(300); ImGui::SmallButton("x=300");

            // Checkbox
            static bool c1=false,c2=false,c3=false,c4=false;
            ImGui::Checkbox("My", &c1); ImGui::SameLine();
            ImGui::Checkbox("Tailor", &c2); ImGui::SameLine();
            ImGui::Checkbox("Is", &c3); ImGui::SameLine();
            ImGui::Checkbox("Rich", &c4);

            // Various
            static float f0=1.0f, f1=2.0f, f2=3.0f;
            ImGui::PushItemWidth(80);
            const char* items[] = { "AAAA", "BBBB", "CCCC", "DDDD" };
            static int item = -1;
            ImGui::Combo("Combo", &item, items, IM_ARRAYSIZE(items)); ImGui::SameLine();
            ImGui::SliderFloat("X", &f0, 0.0f,5.0f); ImGui::SameLine();
            ImGui::SliderFloat("Y", &f1, 0.0f,5.0f); ImGui::SameLine();
            ImGui::SliderFloat("Z", &f2, 0.0f,5.0f);
            ImGui::PopItemWidth();

            ImGui::PushItemWidth(80);
            ImGui::Text("Lists:");
            static int selection[4] = { 0, 1, 2, 3 };
            for (int i = 0; i < 4; i++)
            {
                if (i > 0) ImGui::SameLine();
                ImGui::PushID(i);
                ImGui::ListBox("", &selection[i], items, IM_ARRAYSIZE(items));
                ImGui::PopID();
                //if (ImGui::IsItemHovered()) ImGui::SetTooltip("ListBox %d hovered", i);
            }
            ImGui::PopItemWidth();

            // Dummy
            ImVec2 sz(30,30);
            ImGui::Button("A", sz); ImGui::SameLine();
            ImGui::Dummy(sz); ImGui::SameLine();
            ImGui::Button("B", sz);

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Groups"))
        {
            ImGui::TextWrapped("(Using ImGui::BeginGroup()/EndGroup() to layout items. BeginGroup() basically locks the horizontal position. EndGroup() bundles the whole group so that you can use functions such as IsItemHovered() on it.)");
            ImGui::BeginGroup();
            {
                ImGui::BeginGroup();
                ImGui::Button("AAA");
                ImGui::SameLine();
                ImGui::Button("BBB");
                ImGui::SameLine();
                ImGui::BeginGroup();
                ImGui::Button("CCC");
                ImGui::Button("DDD");
                ImGui::EndGroup();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Group hovered");
                ImGui::SameLine();
                ImGui::Button("EEE");
                ImGui::EndGroup();
            }
            // Capture the group size and create widgets using the same size
            ImVec2 size = ImGui::GetItemRectSize();
            const float values[5] = { 0.5f, 0.20f, 0.80f, 0.60f, 0.25f };
            ImGui::PlotHistogram("##values", values, IM_ARRAYSIZE(values), 0, NULL, 0.0f, 1.0f, size);

            ImGui::Button("ACTION", ImVec2((size.x - ImGui::GetStyle().ItemSpacing.x)*0.5f,size.y));
            ImGui::SameLine();
            ImGui::Button("REACTION", ImVec2((size.x - ImGui::GetStyle().ItemSpacing.x)*0.5f,size.y));
            ImGui::EndGroup();
            ImGui::SameLine();

            ImGui::Button("LEVERAGE\nBUZZWORD", size);
            ImGui::SameLine();

            ImGui::ListBoxHeader("List", size);
            ImGui::Selectable("Selected", true);
            ImGui::Selectable("Not Selected", false);
            ImGui::ListBoxFooter();

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Text Baseline Alignment"))
        {
            ImGui::TextWrapped("(This is testing the vertical alignment that occurs on text to keep it at the same baseline as widgets. Lines only composed of text or \"small\" widgets fit in less vertical spaces than lines with normal widgets)");

            ImGui::Text("One\nTwo\nThree"); ImGui::SameLine();
            ImGui::Text("Hello\nWorld"); ImGui::SameLine();
            ImGui::Text("Banana");

            ImGui::Text("Banana"); ImGui::SameLine();
            ImGui::Text("Hello\nWorld"); ImGui::SameLine();
            ImGui::Text("One\nTwo\nThree");

            ImGui::Button("HOP##1"); ImGui::SameLine();
            ImGui::Text("Banana"); ImGui::SameLine();
            ImGui::Text("Hello\nWorld"); ImGui::SameLine();
            ImGui::Text("Banana");

            ImGui::Button("HOP##2"); ImGui::SameLine();
            ImGui::Text("Hello\nWorld"); ImGui::SameLine();
            ImGui::Text("Banana");

            ImGui::Button("TEST##1"); ImGui::SameLine();
            ImGui::Text("TEST"); ImGui::SameLine();
            ImGui::SmallButton("TEST##2");

            ImGui::AlignFirstTextHeightToWidgets(); // If your line starts with text, call this to align it to upcoming widgets.
            ImGui::Text("Text aligned to Widget"); ImGui::SameLine();
            ImGui::Button("Widget##1"); ImGui::SameLine();
            ImGui::Text("Widget"); ImGui::SameLine();
            ImGui::SmallButton("Widget##2");

            // Tree
            const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            ImGui::Button("Button##1");
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::TreeNode("Node##1")) { for (int i = 0; i < 6; i++) ImGui::BulletText("Item %d..", i); ImGui::TreePop(); }    // Dummy tree data

            ImGui::AlignFirstTextHeightToWidgets();         // Vertically align text node a bit lower so it'll be vertically centered with upcoming widget. Otherwise you can use SmallButton (smaller fit).
            bool node_open = ImGui::TreeNode("Node##2");  // Common mistake to avoid: if we want to SameLine after TreeNode we need to do it before we add child content.
            ImGui::SameLine(0.0f, spacing); ImGui::Button("Button##2");
            if (node_open) { for (int i = 0; i < 6; i++) ImGui::BulletText("Item %d..", i); ImGui::TreePop(); }   // Dummy tree data

            // Bullet
            ImGui::Button("Button##3");
            ImGui::SameLine(0.0f, spacing);
            ImGui::BulletText("Bullet text");

            ImGui::AlignFirstTextHeightToWidgets();
            ImGui::BulletText("Node");
            ImGui::SameLine(0.0f, spacing); ImGui::Button("Button##4");

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Scrolling"))
        {
            ImGui::TextWrapped("(Use SetScrollHere() or SetScrollFromPosY() to scroll to a given position.)");
            static bool track = true;
            static int track_line = 50, scroll_to_px = 200;
            ImGui::Checkbox("Track", &track);
            ImGui::PushItemWidth(100);
            ImGui::SameLine(130); track |= ImGui::DragInt("##line", &track_line, 0.25f, 0, 99, "Line %.0f");
            bool scroll_to = ImGui::Button("Scroll To");
            ImGui::SameLine(130); scroll_to |= ImGui::DragInt("##pos_y", &scroll_to_px, 1.00f, 0, 9999, "y = %.0f px");
            ImGui::PopItemWidth();
            if (scroll_to) track = false;

            for (int i = 0; i < 5; i++)
            {
                if (i > 0) ImGui::SameLine();
                ImGui::BeginGroup();
                ImGui::Text("%s", i == 0 ? "Top" : i == 1 ? "25%" : i == 2 ? "Center" : i == 3 ? "75%" : "Bottom");
                ImGui::BeginChild(ImGui::GetID((void*)(intptr_t)i), ImVec2(ImGui::GetWindowWidth() * 0.17f, 200.0f), true);
                if (scroll_to)
                    ImGui::SetScrollFromPosY(ImGui::GetCursorStartPos().y + scroll_to_px, i * 0.25f);
                for (int line = 0; line < 100; line++)
                {
                    if (track && line == track_line)
                    {
                        ImGui::TextColored(ImColor(255,255,0), "Line %d", line);
                        ImGui::SetScrollHere(i * 0.25f); // 0.0f:top, 0.5f:center, 1.0f:bottom
                    }
                    else
                    {
                        ImGui::Text("Line %d", line);
                    }
                }
                ImGui::EndChild();
                ImGui::EndGroup();
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Horizontal Scrolling"))
        {
            ImGui::Bullet(); ImGui::TextWrapped("Horizontal scrolling for a window has to be enabled explicitly via the ImGuiWindowFlags_HorizontalScrollbar flag.");
            ImGui::Bullet(); ImGui::TextWrapped("You may want to explicitly specify content width by calling SetNextWindowContentWidth() before Begin().");
            static int lines = 7;
            ImGui::SliderInt("Lines", &lines, 1, 15);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2.0f, 1.0f));
            ImGui::BeginChild("scrolling", ImVec2(0, ImGui::GetItemsLineHeightWithSpacing()*7 + 30), true, ImGuiWindowFlags_HorizontalScrollbar);
            for (int line = 0; line < lines; line++)
            {
                // Display random stuff
                int num_buttons = 10 + ((line & 1) ? line * 9 : line * 3);
                for (int n = 0; n < num_buttons; n++)
                {
                    if (n > 0) ImGui::SameLine();
                    ImGui::PushID(n + line * 1000);
                    char num_buf[16];
                    const char* label = (!(n%15)) ? "FizzBuzz" : (!(n%3)) ? "Fizz" : (!(n%5)) ? "Buzz" : (sprintf(num_buf, "%d", n), num_buf);
                    float hue = n*0.05f;
                    ImGui::PushStyleColor(ImGuiCol_Button, ImColor::HSV(hue, 0.6f, 0.6f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImColor::HSV(hue, 0.7f, 0.7f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImColor::HSV(hue, 0.8f, 0.8f));
                    ImGui::Button(label, ImVec2(40.0f + sinf((float)(line + n)) * 20.0f, 0.0f));
                    ImGui::PopStyleColor(3);
                    ImGui::PopID();
                }
            }
            ImGui::EndChild();
            ImGui::PopStyleVar(2);
            float scroll_x_delta = 0.0f;
            ImGui::SmallButton("<<"); if (ImGui::IsItemActive()) scroll_x_delta = -ImGui::GetIO().DeltaTime * 1000.0f;
            ImGui::SameLine(); ImGui::Text("Scroll from code"); ImGui::SameLine();
            ImGui::SmallButton(">>"); if (ImGui::IsItemActive()) scroll_x_delta = +ImGui::GetIO().DeltaTime * 1000.0f;
            if (scroll_x_delta != 0.0f)
            {
                ImGui::BeginChild("scrolling"); // Demonstrate a trick: you can use Begin to set yourself in the context of another window (here we are already out of your child window)
                ImGui::SetScrollX(ImGui::GetScrollX() + scroll_x_delta);
                ImGui::End();
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Clipping"))
        {
            static ImVec2 size(100, 100), offset(50, 20);
            ImGui::TextWrapped("On a per-widget basis we are occasionally clipping text CPU-side if it won't fit in its frame. Otherwise we are doing coarser clipping + passing a scissor rectangle to the renderer. The system is designed to try minimizing both execution and CPU/GPU rendering cost.");
            ImGui::DragFloat2("size", (float*)&size, 0.5f, 0.0f, 200.0f, "%.0f");
            ImGui::TextWrapped("(Click and drag)");
            ImVec2 pos = ImGui::GetCursorScreenPos();
            ImVec4 clip_rect(pos.x, pos.y, pos.x+size.x, pos.y+size.y);
            ImGui::InvisibleButton("##dummy", size);
            if (ImGui::IsItemActive() && ImGui::IsMouseDragging()) { offset.x += ImGui::GetIO().MouseDelta.x; offset.y += ImGui::GetIO().MouseDelta.y; }
            ImGui::GetWindowDrawList()->AddRectFilled(pos, ImVec2(pos.x+size.x,pos.y+size.y), ImColor(90,90,120,255));
            ImGui::GetWindowDrawList()->AddText(ImGui::GetFont(), ImGui::GetFontSize()*2.0f, ImVec2(pos.x+offset.x,pos.y+offset.y), ImColor(255,255,255,255), "Line 1 hello\nLine 2 clip me!", NULL, 0.0f, &clip_rect);
            ImGui::TreePop();
        }
    }

    if (ImGui::CollapsingHeader("Keyboard, Mouse & Focus"))
    {
        if (ImGui::TreeNode("Tabbing"))
        {
            ImGui::Text("Use TAB/SHIFT+TAB to cycle through keyboard editable fields.");
            static char buf[32] = "dummy";
            ImGui::InputText("1", buf, IM_ARRAYSIZE(buf));
            ImGui::InputText("2", buf, IM_ARRAYSIZE(buf));
            ImGui::InputText("3", buf, IM_ARRAYSIZE(buf));
            ImGui::PushAllowKeyboardFocus(false);
            ImGui::InputText("4 (tab skip)", buf, IM_ARRAYSIZE(buf));
            //ImGui::SameLine(); ShowHelperMarker("Use ImGui::PushAllowKeyboardFocus(bool)\nto disable tabbing through certain widgets.");
            ImGui::PopAllowKeyboardFocus();
            ImGui::InputText("5", buf, IM_ARRAYSIZE(buf));
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Focus from code"))
        {
            bool focus_1 = ImGui::Button("Focus on 1"); ImGui::SameLine();
            bool focus_2 = ImGui::Button("Focus on 2"); ImGui::SameLine();
            bool focus_3 = ImGui::Button("Focus on 3");
            int has_focus = 0;
            static char buf[128] = "click on a button to set focus";

            if (focus_1) ImGui::SetKeyboardFocusHere();
            ImGui::InputText("1", buf, IM_ARRAYSIZE(buf));
            if (ImGui::IsItemActive()) has_focus = 1;

            if (focus_2) ImGui::SetKeyboardFocusHere();
            ImGui::InputText("2", buf, IM_ARRAYSIZE(buf));
            if (ImGui::IsItemActive()) has_focus = 2;

            ImGui::PushAllowKeyboardFocus(false);
            if (focus_3) ImGui::SetKeyboardFocusHere();
            ImGui::InputText("3 (tab skip)", buf, IM_ARRAYSIZE(buf));
            if (ImGui::IsItemActive()) has_focus = 3;
            ImGui::PopAllowKeyboardFocus();
            if (has_focus)
                ImGui::Text("Item with focus: %d", has_focus);
            else
                ImGui::Text("Item with focus: <none>");
            ImGui::TextWrapped("Cursor & selection are preserved when refocusing last used item in code.");
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Dragging"))
        {
            ImGui::TextWrapped("You can use ImGui::GetItemActiveDragDelta() to query for the dragged amount on any widget.");
            ImGui::Button("Drag Me");
            if (ImGui::IsItemActive())
            {
                // Draw a line between the button and the mouse cursor
                ImDrawList* draw_list = ImGui::GetWindowDrawList();
                draw_list->PushClipRectFullScreen();
                draw_list->AddLine(ImGui::CalcItemRectClosestPoint(ImGui::GetIO().MousePos, true, -2.0f), ImGui::GetIO().MousePos, ImColor(ImGui::GetStyle().Colors[ImGuiCol_Button]), 4.0f);
                draw_list->PopClipRect();
                ImVec2 value_raw = ImGui::GetMouseDragDelta(0, 0.0f);
                ImVec2 value_with_lock_threshold = ImGui::GetMouseDragDelta(0);
                ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
                ImGui::SameLine(); ImGui::Text("Raw (%.1f, %.1f), WithLockThresold (%.1f, %.1f), MouseDelta (%.1f, %.1f)", value_raw.x, value_raw.y, value_with_lock_threshold.x, value_with_lock_threshold.y, mouse_delta.x, mouse_delta.y);
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Keyboard & Mouse State"))
        {
            ImGuiIO& io = ImGui::GetIO();

            ImGui::Text("MousePos: (%g, %g)", io.MousePos.x, io.MousePos.y);
            ImGui::Text("Mouse down:");     for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (io.MouseDownDuration[i] >= 0.0f)   { ImGui::SameLine(); ImGui::Text("b%d (%.02f secs)", i, io.MouseDownDuration[i]); }
            ImGui::Text("Mouse clicked:");  for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (ImGui::IsMouseClicked(i))          { ImGui::SameLine(); ImGui::Text("b%d", i); }
            ImGui::Text("Mouse dbl-clicked:"); for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (ImGui::IsMouseDoubleClicked(i)) { ImGui::SameLine(); ImGui::Text("b%d", i); }
            ImGui::Text("Mouse released:"); for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (ImGui::IsMouseReleased(i))         { ImGui::SameLine(); ImGui::Text("b%d", i); }
            ImGui::Text("MouseWheel: %.1f", io.MouseWheel);

            ImGui::Text("Keys down:");      for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (io.KeysDownDuration[i] >= 0.0f)     { ImGui::SameLine(); ImGui::Text("%d (%.02f secs)", i, io.KeysDownDuration[i]); }
            ImGui::Text("Keys pressed:");   for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (ImGui::IsKeyPressed(i))             { ImGui::SameLine(); ImGui::Text("%d", i); }
            ImGui::Text("Keys release:");   for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (ImGui::IsKeyReleased(i))            { ImGui::SameLine(); ImGui::Text("%d", i); }
            ImGui::Text("KeyMods: %s%s%s%s", io.KeyCtrl ? "CTRL " : "", io.KeyShift ? "SHIFT " : "", io.KeyAlt ? "ALT " : "", io.KeySuper ? "SUPER " : "");

            ImGui::Text("WantCaptureMouse: %s", io.WantCaptureMouse ? "true" : "false");
            ImGui::Text("WantCaptureKeyboard: %s", io.WantCaptureKeyboard ? "true" : "false");
            ImGui::Text("WantTextInput: %s", io.WantTextInput ? "true" : "false");

            ImGui::Button("Hovering me sets the\nkeyboard capture flag");
            if (ImGui::IsItemHovered())
                ImGui::CaptureKeyboardFromApp(true);
            ImGui::SameLine();
            ImGui::Button("Holding me clears the\nthe keyboard capture flag");
            if (ImGui::IsItemActive())
                ImGui::CaptureKeyboardFromApp(false);

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Mouse cursors"))
        {
            ImGui::TextWrapped("Your application can render a different mouse cursor based on what ImGui::GetMouseCursor() returns. You can also set io.MouseDrawCursor to ask ImGui to render the cursor for you in software.");
            ImGui::Checkbox("io.MouseDrawCursor", &ImGui::GetIO().MouseDrawCursor);
            ImGui::Text("Hover to see mouse cursors:");
            for (int i = 0; i < ImGuiMouseCursor_Count_; i++)
            {
                char label[32];
                sprintf(label, "Mouse cursor %d", i);
                ImGui::Bullet(); ImGui::Selectable(label, false);
                if (ImGui::IsItemHovered())
                    ImGui::SetMouseCursor(i);
            }
            ImGui::TreePop();
        }
    }
*/

	m_rootNode.Render(viewer);
/*
	if (ImGui::CollapsingHeader("Graphs widgets")) {
		static bool animate = true;
		ImGui::Checkbox("Animate", &animate);

		static float arr[] = { 0.6f, 0.1f, 1.0f, 0.5f, 0.92f, 0.1f, 0.2f };
		ImGui::PlotLines("Frame Times", arr, IM_ARRAYSIZE(arr));

		// Create a dummy array of contiguous float values to plot
		// Tip: If your float aren't contiguous but part of a structure, you can pass a pointer to your first float and the sizeof() of your structure in the Stride parameter.
		static float values[90] = { 0 };
		static int values_offset = 0;
		if (animate) {
			static float refresh_time = ImGui::GetTime(); // Create dummy data at fixed 60 hz rate for the demo
			for (; ImGui::GetTime() > refresh_time + 1.0f / 60.0f; refresh_time += 1.0f / 60.0f) {
				static float phase = 0.0f;
				values[values_offset] = cosf(phase);
				values_offset = (values_offset + 1) % IM_ARRAYSIZE(values);
				phase += 0.10f*values_offset;
			}
		}
		ImGui::PlotLines("Lines", values, IM_ARRAYSIZE(values), values_offset, "avg 0.0", -1.0f, 1.0f, ImVec2(0, 80));
		ImGui::PlotHistogram("Histogram", arr, IM_ARRAYSIZE(arr), 0, NULL, 0.0f, 1.0f, ImVec2(0, 80));

		// Use functions to generate output
		// FIXME: This is rather awkward because current plot API only pass in indices. We probably want an API passing floats and user provide sample0 rate/count.
		struct Funcs
		{
			static float Sin(void*, int i) { return sinf(i * 0.1f); }
			static float Saw(void*, int i) { return (i & 1) ? 1.0f : 0.0f; }
		};
		static int func_type = 0, display_count = 70;
		ImGui::Separator();
		ImGui::PushItemWidth(100); ImGui::Combo("func", &func_type, "Sin\0Saw\0"); ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::SliderInt("Sample count", &display_count, 1, 400);
		float(*func)(void*, int) = (func_type == 0) ? Funcs::Sin : Funcs::Saw;
		ImGui::PlotLines("Lines", func, NULL, display_count, 0, NULL, -1.0f, 1.0f, ImVec2(0, 80));
		ImGui::PlotHistogram("Histogram", func, NULL, display_count, 0, NULL, -1.0f, 1.0f, ImVec2(0, 80));
		ImGui::Separator();

		// Animate a simple progress bar
		static float progress = 0.0f, progress_dir = 1.0f;
		if (animate) {
			progress += progress_dir * 0.4f * ImGui::GetIO().DeltaTime;
			if (progress >= +1.1f) { progress = +1.1f; progress_dir *= -1.0f; }
			if (progress <= -0.1f) { progress = -0.1f; progress_dir *= -1.0f; }
		}

		// Typically we would use ImVec2(-1.0f,0.0f) to use all available width, or ImVec2(width,0.0f) for a specified width. ImVec2(0.0f,0.0f) uses ItemWidth.
		ImGui::ProgressBar(progress, ImVec2(0.0f, 0.0f));
		ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
		ImGui::Text("Progress Bar");

		float progress_saturated = (progress < 0.0f) ? 0.0f : (progress > 1.0f) ? 1.0f : progress;
		char buf[32];
		sprintf(buf, "%d/%d", (int)(progress_saturated * 1753), 1753);
		ImGui::ProgressBar(progress, ImVec2(0.f, 0.f), buf);
	}
*/
}


dProfilerTrace::dTraceCapture::dTraceCapture()
	:m_treads(dArray<dTrackerThread*>())
	,m_minTime(0.0f)
	,m_maxTime(0.0f)
	,m_windowSize(0.0f)
	,m_scale(1.0f)
{
}

dProfilerTrace::dTraceCapture::~dTraceCapture()
{
	for (int i = 0; i < m_treads.GetSize(); i++) {
		delete m_treads[i];
	}
}

void dProfilerTrace::dTraceCapture::DrawTimeLine()
{
	ImGuiIO& io = ImGui::GetIO();
	ImGui::Text("time line");
/*
	ImDrawList* const draw = ImGui::GetWindowDrawList();
	ImVec2 p0(ImGui::GetCursorScreenPos());
	ImVec2 size(ImGui::GetWindowSize());
	ImVec2 p1(p0);
	p1.x += size.x - 40.0f;
	draw->AddLine(p0, p1, IM_COL32_A_MASK + 0x8040);
	m_timeLineP0 = p0.x;
	m_timeLineP1 = p1.x;

	ImVec2 q0(p0);
	q0.x = m_timeLinePosition - m_timeWidth * 0.5f;
	q0.y -= 5.0f;

	ImVec2 q1(q0);
	q1.x += m_timeWidth;
	q1.y += 10.0f;

	draw->AddRectFilled(q0, q1, IM_COL32_A_MASK + 0x8040);
*/
	ImGui::Text("");
}

void dProfilerTrace::dTraceCapture::MouseMove()
{
/*
	switch (m_timeLineState) 
	{
		case 0:
		{
			if (ImGui::IsMouseDown(0)) {
				ImVec2 mousePos(ImGui::GetMousePos());
				//m_timeLinePosition = dClamp(mousePos.x, p0.x + m_timeWidth * 0.5f, p1.x - m_timeWidth * 0.5f);
				//q0.x = m_timeLinePosition - m_timeWidth * 0.5f;
				//q1.x = q0.x + m_timeWidth;
			} else {
				m_timeLineState = -1;
			}

			break;
		}

		default:
		{
		   if (ImGui::IsMouseDown(0) && ImGui::IsMouseHoveringRect(m_mouseBoxp0, m_mouseBoxp1, false)) {
			   ImVec2 mousePos(ImGui::GetMousePos());
			   m_timeLineOffset = ;
			   m_timeLineState = 0;
		   }
		}
	}
*/
}


void dProfilerTrace::dTraceCapture::Render(dTimeTrackerViewer* const viewer)
{
	ImVec2 size(ImGui::GetWindowSize());
	m_windowSize = size.x;

	// display time line
	DrawTimeLine();

	// render all traces
	for (int i = 0; i < m_treads.GetSize(); i++) {
		m_treads[i]->Render(viewer);
	}

	// update mouse motion
	MouseMove();
}
