```cpp
#define _CRT_SECURE_NO_WARNINGS // 防止 Windows 安全警告

/*********************************************************************
 * OMPL Custom Planner Demo
 * * 功能：
 * 1. 实现一个自定义的规划器 "MyRRTPlanner" (基于 RRT 原理)。
 * 2. 在一个带有圆形障碍物的 2D 空间中进行规划。
 * 3. 演示如何继承 ompl::base::Planner 并实现 solve() 函数。
 * * 修复：
 * - 修正了 RNG 类的命名空间 (ompl::RNG) 和头文件引用。
 *********************************************************************/

#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/SimpleSetup.h>
 // [关键修复] 引入随机数生成器头文件
#include <ompl/util/RandomNumbers.h> 
#include <ompl/config.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// ==================================================================================
// 第一部分：自定义规划器实现 (MyRRTPlanner)
// ==================================================================================

class MyRRTPlanner : public ob::Planner
{
public:
    // 构造函数：必须初始化基类
    MyRRTPlanner(const ob::SpaceInformationPtr& si) : ob::Planner(si, "MyRRTPlanner")
    {
        // 设置规划器能力说明书 (Specs)
        specs_.approximateSolutions = true; // 支持近似解
        specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION; // 支持可采样的目标区域
        specs_.directed = true; // 这是一个有向树算法
    }

    // 析构函数：清理内存
    virtual ~MyRRTPlanner()
    {
        clear();
    }

    // --- 1. 初始化设置 ---
    void setup() override
    {
        Planner::setup();
        // 如果还没有采样器，就创建一个
        if (!sampler_)
            sampler_ = si_->allocStateSampler();
    }

    // --- 2. 清理内存 ---
    void clear() override
    {
        Planner::clear();
        // 释放树中所有节点的内存
        for (auto& motion : tree_)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
        tree_.clear();
        sampler_.reset();
    }

    // --- 3. 核心求解函数 ---
    ob::PlannerStatus solve(const ob::PlannerTerminationCondition& ptc) override
    {
        // 1. 检查准备工作
        checkValidity();

        // 获取目标定义
        ob::Goal* goal = pdef_->getGoal().get();
        auto* goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

        // 2. 获取并添加起点
        // pis_ (PlannerInputStates) 是基类提供的助手，用于管理起点
        while (const ob::State* st = pis_.nextStart())
        {
            Motion* motion = new Motion(si_);
            si_->copyState(motion->state, st);
            tree_.push_back(motion);
        }

        if (tree_.empty())
        {
            OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return ob::PlannerStatus::INVALID_START;
        }

        // 准备临时变量
        ob::State* rstate = si_->allocState(); // 随机采样点
        ob::State* xstate = si_->allocState(); // 延伸出的新点
        Motion* solution = nullptr;
        double approxDist = std::numeric_limits<double>::infinity(); // 记录离目标的最近距离

        std::cout << "MyRRTPlanner: Starting planning with " << tree_.size() << " start states..." << std::endl;

        // 3. 主循环
        while (ptc == false)
        {
            // a. 采样：在空间中随机撒一个点 rstate
            // (高级用法：也可以以一定概率向目标采样)
            if (goal_s && rng_.uniform01() < 0.05 && goal_s->canSample())
                goal_s->sampleGoal(rstate);
            else
                sampler_->sampleUniform(rstate);

            // b. 寻找最近邻：在树上找到离 rstate 最近的节点 (nmotion)
            // 这里为了简单演示，使用线性遍历 (O(N))。
            // 实际生产环境应使用 NearestNeighbors (KD-Tree等) 加速。
            Motion* nmotion = tree_[0];
            double minDesc = si_->distance(nmotion->state, rstate);
            for (auto& m : tree_)
            {
                double d = si_->distance(m->state, rstate);
                if (d < minDesc)
                {
                    minDesc = d;
                    nmotion = m;
                }
            }

            // c. 延伸 (Steer)：从 nmotion 朝 rstate 走一步
            // 我们限制最大步长为 0.5 (range)
            double d = si_->distance(nmotion->state, rstate);
            if (d > 0.5)
            {
                si_->getStateSpace()->interpolate(nmotion->state, rstate, 0.5 / d, xstate);
            }
            else
            {
                si_->copyState(xstate, rstate);
            }

            // d. 碰撞检测：检查从 nmotion 到 xstate 的路径是否安全
            if (si_->checkMotion(nmotion->state, xstate))
            {
                // 有效！创建新节点加入树
                Motion* motion = new Motion(si_);
                si_->copyState(motion->state, xstate);
                motion->parent = nmotion;
                tree_.push_back(motion);

                // 更新“最近距离”，用于近似解
                double dist = 0.0;
                bool satisfied = goal->isSatisfied(motion->state, &dist);
                if (dist < approxDist)
                {
                    approxDist = dist;
                    // 如果这是近似解，我们也先记下来
                    // (在完全解找到之前，它就是最好的)
                }

                // e. 检查是否到达终点
                if (satisfied)
                {
                    approxDist = dist;
                    solution = motion;
                    break; // 找到了！退出循环
                }
            }
        }

        // 4. 构建并保存路径
        bool solved = false;
        bool approximate = false;

        if (solution == nullptr)
        {
            // 没找到完全解，看看有没有近似解
            solution = tree_[0]; // 简单回落
            approximate = true;
        }
        else
        {
            solved = true;
        }

        if (solution != nullptr)
        {
            // 回溯路径：从终点往回找父节点，直到起点
            auto path(std::make_shared<og::PathGeometric>(si_));
            for (Motion* m = solution; m != nullptr; m = m->parent)
            {
                path->append(m->state);
            }
            path->reverse(); // 反转，变成 起点 -> 终点

            // 将路径存入 ProblemDefinition
            pdef_->addSolutionPath(path, approximate, approxDist, getName());
        }

        // 清理临时状态内存
        si_->freeState(rstate);
        si_->freeState(xstate);

        return solved ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
    }

    // --- 4. 调试数据接口 ---
    void getPlannerData(ob::PlannerData& data) const override
    {
        Planner::getPlannerData(data);
        for (auto& motion : tree_)
        {
            if (motion->parent)
                data.addEdge(ob::PlannerDataVertex(motion->parent->state),
                    ob::PlannerDataVertex(motion->state));
            else
                data.addStartVertex(ob::PlannerDataVertex(motion->state));
        }
    }

protected:
    // 内部结构体：树的节点
    struct Motion
    {
        ob::State* state;
        Motion* parent;

        Motion(const ob::SpaceInformationPtr& si) : state(si->allocState()), parent(nullptr) {}
    };

    // 成员变量
    ob::StateSamplerPtr sampler_; // 采样器
    std::vector<Motion*> tree_;  // 搜索树 (节点列表)
    // [关键修复] RNG 是 ompl 命名空间下的，不是 ompl::base
    ompl::RNG rng_;               // 随机数生成器
};

// ==================================================================================
// 第二部分：规划问题定义
// ==================================================================================

// 碰撞检测：定义一个圆形障碍物
// 圆心 (0.5, 0.5), 半径 0.25
bool isStateValid(const ob::State* state)
{
    const auto* pos = state->as<ob::RealVectorStateSpace::StateType>();
    double x = pos->values[0];
    double y = pos->values[1];

    // 计算到圆心的距离
    double dist = std::sqrt((x - 0.5) * (x - 0.5) + (y - 0.5) * (y - 0.5));

    // 如果距离大于半径，则有效（无碰撞）
    return dist > 0.25;
}

int main()
{
    std::cout << "Starting OMPL Custom Planner Demo..." << std::endl;

    // 1. 创建状态空间：2D 平面 [0, 1] x [0, 1]
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    space->setBounds(0.0, 1.0);

    // 2. 创建 SimpleSetup
    og::SimpleSetup ss(space);

    // 3. 设置碰撞检测
    ss.setStateValidityChecker(isStateValid);

    // 4. 设置起点 (0, 0) 和 终点 (1, 1)
    ob::ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 1.0;
    goal[1] = 1.0;

    ss.setStartAndGoalStates(start, goal);

    // 5. [关键] 使用我们的自定义规划器
    // 创建 MyRRTPlanner 实例
    auto myPlanner = std::make_shared<MyRRTPlanner>(ss.getSpaceInformation());

    // 设置给 SimpleSetup
    ss.setPlanner(myPlanner);

    // 6. 求解
    // 给它 1.0 秒时间
    // [注意] 使用 timedPlannerTerminationCondition 避免类型歧义
    ob::PlannerStatus solved = ss.solve(ob::timedPlannerTerminationCondition(1.0));

    if (solved)
    {
        std::cout << "Found solution!" << std::endl;

        // 获取并打印路径
        auto path = ss.getSolutionPath();
        path.interpolate(20); // 插值让路径更好看

        // 打印到控制台 (实际使用时可保存到文件)
        // path.printAsMatrix(std::cout);

        std::cout << "Path length: " << path.length() << std::endl;
        std::cout << "Custom planner works successfully!" << std::endl;
    }
    else
    {
        std::cout << "No solution found." << std::endl;
    }

    return 0;
}
```