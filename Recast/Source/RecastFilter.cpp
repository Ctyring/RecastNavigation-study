//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Recast.h"
#include "RecastAssert.h"

#include <stdlib.h>

void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if (!walkable && previousWasWalkable)
				{
                    // 上表面差小于可攀爬高度，即可标记悬空的span为可行走区域
					if (rcAbs((int)span->smax - (int)previousSpan->smax) <= walkableClimb)
					{
						span->area = previousArea;
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWasWalkable = walkable;
				previousArea = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb,
                        rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
	
	// Mark border spans.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non walkable spans.
                // 跳过不可行走的span
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

                const int bot = (int)(span->smax);
                const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;

                // 下面两种情况时，会认为当前span为不可行走,下面的遍历检查了以下两种情况
                // a：当nspan比span低walkableClimb以上时，说明span走向nspan时有落差，则span置位RC_NULL_AREA
                // b：当span的所有nspan之间的高度差相差walkableClimb以上时，则认为比较陡峭，span置位RC_NULL_AREA

				// Find neighbours minimum height.
                // 寻找邻居的最小高度 判断条件a
				int minNeighborHeight = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
                // 可访问邻居的最小和最大高度 判断条件b
				int accessibleNeighborMinHeight = span->smax;
				int accessibleNeighborMaxHeight = span->smax;

                // 遍历四个邻居
				for (int direction = 0; direction < 4; ++direction)
				{
					int dx = x + rcGetDirOffsetX(direction);
					int dy = z + rcGetDirOffsetY(direction);
					// Skip neighbours which are out of bounds.
                    // 跳过超出边界的邻居
					if (dx < 0 || dy < 0 || dx >= xSize || dy >= zSize)
					{
                        // 设置最小高度，这样就可以确保一定会被过滤
						minNeighborHeight = rcMin(minNeighborHeight, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					const rcSpan* neighborSpan = heightfield.spans[dx + dy * xSize];
					int neighborBot = -walkableClimb;
					int neighborTop = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHT;
					
					// Skip neighbour if the gap between the spans is too small.
                    // 间隙小的情况会在其他过滤器处理
					if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
					{
                        // 计算最小高度
						minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);
					}

					// Rest of the spans.
                    // 遍历对应方向邻居的y轴整个链表
					for (neighborSpan = heightfield.spans[dx + dy * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
					{
						neighborBot = (int)neighborSpan->smax;
						neighborTop = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHT;
						
						// Skip neighbour if the gap between the spans is too small.
						if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
						{
                            // 情况a
							minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);

							// Find min/max accessible neighbour height.
                            // 计算邻居的最小和最大高度
							if (rcAbs(neighborBot - bot) <= walkableClimb)
							{
								if (neighborBot < accessibleNeighborMinHeight) accessibleNeighborMinHeight = neighborBot;
								if (neighborBot > accessibleNeighborMaxHeight) accessibleNeighborMaxHeight = neighborBot;
							}

						}
					}
				}

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
                // 情况a
				if (minNeighborHeight < -walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				// 情况b
                else if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);
	
	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;
                // 相邻两个span之间间隙过小，直接过滤
				if ((top - bot) <= walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
