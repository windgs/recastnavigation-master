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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Sample.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

unsigned int SampleDebugDraw::areaToCol(unsigned int area)
{
	switch(area)
	{
	// Ground (0) : light blue
	case SAMPLE_POLYAREA_GROUND: return duRGBA(0, 192, 255, 255);
	// Water : blue
	case SAMPLE_POLYAREA_WATER: return duRGBA(0, 0, 255, 255);
	// Road : brown
	case SAMPLE_POLYAREA_ROAD: return duRGBA(50, 20, 12, 255);
	// Door : cyan
	case SAMPLE_POLYAREA_DOOR: return duRGBA(0, 255, 255, 255);
	// Grass : green
	case SAMPLE_POLYAREA_GRASS: return duRGBA(0, 255, 0, 255);
	// Jump : yellow
	case SAMPLE_POLYAREA_JUMP: return duRGBA(255, 255, 0, 255);
	// Unexpected : red
	default: return duRGBA(255, 0, 0, 255);
	}
}

Sample::Sample() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS|DU_DRAWNAVMESH_CLOSEDLIST),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_tool(0),
	m_ctx(0)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();

	for (int i = 0; i < MAX_TOOLS; i++)
		m_toolStates[i] = 0;
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	dtFreeCrowd(m_crowd);
	delete m_tool;
	for (int i = 0; i < MAX_TOOLS; i++)
		delete m_toolStates[i];
}

void Sample::setTool(SampleTool* tool)  
{
	delete m_tool;
	m_tool = tool;
	if (tool)
 		m_tool->init(this);
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender()
{
	if (!m_geom)
		return;
	
	// Draw mesh
	duDebugDrawTriMesh(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
					   m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0, 1.0f);
	// Draw bounds
	const float* bmin = m_geom->getMeshBoundsMin();
	const float* bmax = m_geom->getMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;

	const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings)
	{
		m_cellSize = buildSettings->cellSize;
		m_cellHeight = buildSettings->cellHeight;
		m_agentHeight = buildSettings->agentHeight;
		m_agentRadius = buildSettings->agentRadius;
		m_agentMaxClimb = buildSettings->agentMaxClimb;
		m_agentMaxSlope = buildSettings->agentMaxSlope;
		m_regionMinSize = buildSettings->regionMinSize;
		m_regionMergeSize = buildSettings->regionMergeSize;
		m_edgeMaxLen = buildSettings->edgeMaxLen;
		m_edgeMaxError = buildSettings->edgeMaxError;
		m_vertsPerPoly = buildSettings->vertsPerPoly;
		m_detailSampleDist = buildSettings->detailSampleDist;
		m_detailSampleMaxError = buildSettings->detailSampleMaxError;
		m_partitionType = buildSettings->partitionType;
	}
}

void Sample::collectSettings(BuildSettings& settings)
{
	settings.cellSize = m_cellSize;
	settings.cellHeight = m_cellHeight;
	settings.agentHeight = m_agentHeight;
	settings.agentRadius = m_agentRadius;
	settings.agentMaxClimb = m_agentMaxClimb;
	settings.agentMaxSlope = m_agentMaxSlope;
	settings.regionMinSize = m_regionMinSize;
	settings.regionMergeSize = m_regionMergeSize;
	settings.edgeMaxLen = m_edgeMaxLen;
	settings.edgeMaxError = m_edgeMaxError;
	settings.vertsPerPoly = m_vertsPerPoly;
	settings.detailSampleDist = m_detailSampleDist;
	settings.detailSampleMaxError = m_detailSampleMaxError;
	settings.partitionType = m_partitionType;
}


void Sample::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.9f;
	m_agentMaxSlope = 45.0f;
	m_regionMinSize = 8;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
}

void Sample::handleCommonSettings()
{
	/*
	����navmesh�ĺ��������Sample_SoloMesh::handleBuild()�С���һ���ǳ�ʼ�����������initialize build config������Ҫ�ļ����У�

	cs��xzƽ�������صĴ�С��
	ch��y�������صĸ߶ȡ�
	walkableSlopeAngle����������б�Ƕȡ�
	walkableHeight��Ѱ·agent�ĸ߶ȡ�
	walkableClimb��Ѱ·agent�����¸߶ȡ�
	walkableRadius��Ѱ·agent�İ뾶��
	��Щ���������ں�������ػ��������õ������õ�ʱ��ϸ˵��
	*/
	//Rasterization�������趨���صĳߴ硣
	imguiLabel("Rasterization-���صĳߴ�");
	//cellSize����С��ֵ�ܹ��������Ӿ�ȷ�ӽ�ԭʼ����ͼ�ε����񣬼��������ɵ���������������ɶ���β����ϴ�ƫ��ȵ����⣬���ǻ����ĸ��Ӷ�Ĵ���ʱ����ڴ�ռ�á�
	imguiSlider("Cell Size-��������voxel��X-Z-plane)�ϵĴ�С", &m_cellSize, 0.1f, 1.0f, 0.01f);
	//cellHeight���ϵ͵�ֵ��������ڴ����Ĳ���������Ӱ�죬�ϵ͵�ֵ��Ȼ�ܹ�ʹ��������ԭʼͼ�Σ���������ǰ�͹��ƽ�ĵ��Σ��ϵ͵�ֵ���ܻ�����ڽӵ�����֮��������ѣ�
	//���±���Ӧ������һ���������ɷ��롣���ýϸ�ʱ�����Կ��������ԭʼͼ�μ��ϴ�
	imguiSlider("Cell Height-��������voxel��Y-plane)�ϵĴ�С", &m_cellHeight, 0.1f, 1.0f, 0.01f);
	
	if (m_geom)
	{
		const float* bmin = m_geom->getNavMeshBoundsMin();
		const float* bmax = m_geom->getNavMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		imguiValue(text);
	}
	
	imguiSeparator();
	//Agent�����������ɫ�����á�	
	imguiLabel("Agent-��ɫ");
	//Height: ��ɫ����ߡ�
	imguiSlider("Height-���", &m_agentHeight, 0.1f, 5.0f, 0.1f);
	//Radius����ɫ�İ뾶��
	imguiSlider("Radius-�뾶", &m_agentRadius, 0.0f, 5.0f, 0.1f);
	//Max Climb����������������¥��ʱ��ÿ��̨�׵ĸ߶ȡ�
	imguiSlider("Max Climb-���¸߶�", &m_agentMaxClimb, 0.1f, 5.0f, 0.1f);
	//Max Slope�����������������µ��¶ȣ��ԽǶ�ֵ�������á�
	imguiSlider("Max Slope-��������б�Ƕ�", &m_agentMaxSlope, 0.0f, 90.0f, 1.0f);
	
	imguiSeparator();
	//����
	imguiLabel("Region-����");
	//��С����ߴ�
	imguiSlider("Min Region Size-��С����ߴ�", &m_regionMinSize, 0.0f, 150.0f, 1.0f);
	//�ϲ�����ߴ�
	imguiSlider("Merged Region Size-�ϲ�����ߴ�", &m_regionMergeSize, 0.0f, 150.0f, 1.0f);

	imguiSeparator();
	//�߶�����뷽ʽ
	imguiLabel("Partitioning-�߶�����뷽ʽ");
	//��ˮ���㷨
	//Watershed ����ˮ���㷨����䡢Ч����ã�������Ƚ�����һ���������ߴ���
	if (imguiCheck("Watershed-��ˮ���㷨", m_partitionType == SAMPLE_PARTITION_WATERSHED))
		m_partitionType = SAMPLE_PARTITION_WATERSHED;
	//Monotone�㷨
	//Monotone ������ҿ��Ա�֤���ɵ��ǲ��ص���û�ж��� Region���������ɵ� Region ���ܻ���ϸ�ֳ���Ч�����á�
	if (imguiCheck("Monotone-Monotone�㷨", m_partitionType == SAMPLE_PARTITION_MONOTONE))
		m_partitionType = SAMPLE_PARTITION_MONOTONE;
	//�ֲ��㷨
	//Layer���ٶȡ�Ч���������ˮ���㷨�� Monotone partioning ֮�䣬�Ƚ������ڳ�ʼ���ݡ�
	if (imguiCheck("Layers-�ֲ��㷨", m_partitionType == SAMPLE_PARTITION_LAYERS))
		m_partitionType = SAMPLE_PARTITION_LAYERS;
	
	imguiSeparator();
	imguiLabel("Filtering");
	if (imguiCheck("Low Hanging Obstacles", m_filterLowHangingObstacles))
		m_filterLowHangingObstacles = !m_filterLowHangingObstacles;
	if (imguiCheck("Ledge Spans", m_filterLedgeSpans))
		m_filterLedgeSpans= !m_filterLedgeSpans;
	if (imguiCheck("Walkable Low Height Spans", m_filterWalkableLowHeightSpans))
		m_filterWalkableLowHeightSpans = !m_filterWalkableLowHeightSpans;

	imguiSeparator();
	//���������
	imguiLabel("Polygonization-���������");
	//������Ǳ߳���
	imguiSlider("Max Edge Length-������Ǳ߳���", &m_edgeMaxLen, 0.0f, 50.0f, 1.0f);
	//�����ݴ�
	imguiSlider("Max Edge Error-�����ݴ�", &m_edgeMaxError, 0.1f, 3.0f, 0.1f);
	//ÿ�������ζ�����
	imguiSlider("Verts Per Poly-ÿ�������ζ�����", &m_vertsPerPoly, 3.0f, 12.0f, 1.0f);		

	imguiSeparator();
	imguiLabel("Detail Mesh-ϸ������");
	//��������
	imguiSlider("Sample Distance-��������", &m_detailSampleDist, 0.0f, 16.0f, 1.0f);
	//�������ݴ�
	imguiSlider("Max Sample Error-�������ݴ�", &m_detailSampleMaxError, 0.0f, 16.0f, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void Sample::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}


void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

//��ȡnavmesh����
struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

dtNavMesh* Sample::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		//add by huyf 2019.02.26:Modify memory leaks 
		dtFreeNavMesh(mesh);
		//add end:Modify memory leaks
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			//add by huyf 2019.02.26:Modify memory leaks 
			dtFreeNavMesh(mesh);
			//add end:Modify memory leaks
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			//add by huyf 2019.02.26:Modify memory leaks 
			dtFreeNavMesh(mesh);
			//add end:Modify memory leaks
			dtFree(data);
			fclose(fp);
			return 0;
		}

		//add by huyf 2019.02.26:Modify memory leaks 
		//mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
		status = mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
		if (dtStatusFailed(status))
		{
			dtFreeNavMesh(mesh);
			dtFree(data);
			fclose(fp);
			return 0;
		}
		//add end:Modify memory leaks 
	}

	fclose(fp);

	return mesh;
}

void Sample::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}
