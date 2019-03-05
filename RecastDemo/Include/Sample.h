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

#ifndef RECASTSAMPLE_H
#define RECASTSAMPLE_H

#include "Recast.h"
#include "SampleInterfaces.h"


/// Tool types.
enum SampleToolType
{
	TOOL_NONE = 0,
	TOOL_TILE_EDIT,				//瓦片编辑
	TOOL_TILE_HIGHLIGHT,		//突出编辑
	TOOL_TEMP_OBSTACLE,			//临时障碍
	TOOL_NAVMESH_TESTER,		//navmesh 测试人员
	TOOL_NAVMESH_PRUNE,			//navmesh修剪
	TOOL_OFFMESH_CONNECTION,	//分离网格链接
	TOOL_CONVEX_VOLUME,			//凸体积
	TOOL_CROWD,					//人群			
	MAX_TOOLS
};

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
/// 这些只是在样本中使用一致值的样本区域。使用应该根据他的需要指定这些。
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,			//地面
	SAMPLE_POLYAREA_WATER,			//水（河流）
	SAMPLE_POLYAREA_ROAD,			//陆地
	SAMPLE_POLYAREA_DOOR,			//传送门
	SAMPLE_POLYAREA_GRASS,			//草坪
	SAMPLE_POLYAREA_JUMP,			//跳点
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road) // 能够行走(地面、草地、道路)
	SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).	//能够游泳(水)。
	SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors. //能够穿透墙
	SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump. //能够跳。
	SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon //禁用多边形-类似于障碍不能跨越-设置整块多边形区域
	SAMPLE_POLYFLAGS_ALL		= 0xffff	// All abilities. //所有的属性。
};

class SampleDebugDraw : public DebugDrawGL
{
public:
	virtual unsigned int areaToCol(unsigned int area);
};

//简单分区类型
enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,		//分水岭算法
	SAMPLE_PARTITION_MONOTONE,		//monotne算法
	SAMPLE_PARTITION_LAYERS,		//层算法
};

//8个继承子类 抽象类
/*
//继承标注：class ConvexVolumeTool : public SampleTool
//继承标注：class CrowdTool : public SampleTool
//继承标注：class NavMeshPruneTool : public SampleTool
//继承标注：class NavMeshTesterTool : public SampleTool
//继承标注：class OffMeshConnectionTool : public SampleTool
//继承标注：class TempObstacleHilightTool : public SampleTool
//继承标注：class TempObstacleCreateTool : public SampleTool
//继承标注：class NavMeshTileTool : public SampleTool
*/
struct SampleTool
{
	virtual ~SampleTool() {}
	virtual int type() = 0;
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClick(const float* s, const float* p, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(const float dt) = 0;
};

//继承标注：class CrowdToolState : public SampleToolState
struct SampleToolState {
	virtual ~SampleToolState() {}
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleUpdate(const float dt) = 0;
};

//样本类
//4个继承子类
/*
class Sample_Debug : public Sample
class Sample_SoloMesh : public Sample
class Sample_TempObstacles : public Sample
class Sample_TileMesh : public Sample
*/
class Sample
{
protected:
	class InputGeom* m_geom;				//几何图形指针
	class dtNavMesh* m_navMesh;				//navMesh指针
	class dtNavMeshQuery* m_navQuery;		//navMeshQuery查询指针
	class dtCrowd* m_crowd;					//人群指针

	unsigned char m_navMeshDrawFlags;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;
	
	SampleTool* m_tool;
	SampleToolState* m_toolStates[MAX_TOOLS];
	
	BuildContext* m_ctx;

	SampleDebugDraw m_dd;
	
	dtNavMesh* loadAll(const char* path);
	void saveAll(const char* path, const dtNavMesh* mesh);

public:
	Sample();
	virtual ~Sample();
	
	void setContext(BuildContext* ctx) { m_ctx = ctx; }
	//add  by huyf
	BuildContext* getContext() { return m_ctx; }
	//add end
	
	void setTool(SampleTool* tool);
	SampleToolState* getToolState(int type) { return m_toolStates[type]; }
	void setToolState(int type, SampleToolState* s) { m_toolStates[type] = s; }

	SampleDebugDraw& getDebugDraw() { return m_dd; }

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);
	virtual void collectSettings(struct BuildSettings& settings);

	virtual class InputGeom* getInputGeom() { return m_geom; }
	virtual class dtNavMesh* getNavMesh() { return m_navMesh; }
	virtual class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	virtual class dtCrowd* getCrowd() { return m_crowd; }
	virtual float getAgentRadius() { return m_agentRadius; }
	virtual float getAgentHeight() { return m_agentHeight; }
	virtual float getAgentClimb() { return m_agentMaxClimb; }
	
	unsigned char getNavMeshDrawFlags() const { return m_navMeshDrawFlags; }
	void setNavMeshDrawFlags(unsigned char flags) { m_navMeshDrawFlags = flags; }

	void updateToolStates(const float dt);
	void initToolStates(Sample* sample);
	void resetToolStates();
	void renderToolStates();
	void renderOverlayToolStates(double* proj, double* model, int* view);

	void resetCommonSettings();
	void handleCommonSettings();

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample(const Sample&);
	Sample& operator=(const Sample&);
};


#endif // RECASTSAMPLE_H
