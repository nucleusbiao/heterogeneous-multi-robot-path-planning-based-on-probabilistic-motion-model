#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/PCBS_ON.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

double time_div=0.2; //wait time
double t_Div=0.02; //time step
double threshold=0.6;//time threshold
double a=0.8;//triangular probability distribution's parameter a
double c=1.2;//triangular probability distribution's parameter c
double probThreshold=0.05;//probability threshold
double velocity=1;//velocity

//generate random number from 0 to randomMax
void generateRandom(double& randomNum){
    randomNum=static_cast<double>(rand())/RAND_MAX;
}

void generateProbBand(std::vector<double>& pdf,
                      std::vector<double>& probBand,
                      std::vector<double>& timeBand,
                      double tDiv){
    probBand.clear();
    timeBand.clear();
    size_t regionL=0;
    size_t regionH=0;
    double tempNum=0;
    for(size_t i=0;i<=pdf.size()-1;i++){
        if(pdf[i]>tDiv*0.1){
            regionL=i;
            break;
        }
    }

    for(size_t i=pdf.size()-1;i!=0;i--){
        if(pdf[i]>tDiv*0.1){
            regionH=i;
            break;
        }
    }

    for(size_t i=regionL;i<=regionH;i++){
        if(i==regionL){
            tempNum=pdf[i];
        }
        else{
            tempNum=tempNum+pdf[i];
        }
        probBand.emplace_back(tempNum*tDiv);
        timeBand.emplace_back(i*tDiv);
    }
}


double generateRandomTime(std::vector<double>& pdf,
                          double tDiv){
    std::vector<double> timeBand;
    std::vector<double> probBand;
    std::vector<int> count;
    double time=3;
    double rand;
    generateProbBand(pdf,probBand,timeBand,tDiv);
    generateRandom(rand);
    for(size_t j=0;j<=(probBand.size()-1);j++){
        if(j==0&&(rand<probBand[j])){
            time=timeBand[j];
        }else if(j==(probBand.size()-1)&&rand>probBand[j-1]){
            time=timeBand[j];
        }
        else{
            if(rand>probBand[j-1]&&rand<probBand[j]){
                time=timeBand[j];
            }
        }

    }
    return time;
}

double min(double a,double b){
    if(a>=b)
        return b;
    else
        return a;
}

double max(double a,double b){
    if(a>=b)
        return a;
    else
        return b;
}


void convdx(std::vector<double>& result,
            const std::vector<double>& a,
            const std::vector<double>& b){
    double temp1,temp2;
    for(size_t i=0;i<a.size()+b.size()-1;i++){
        temp1=0;
        temp2=0;
        for(size_t j=0;j<a.size();j++){
            if(i<j||i-j>(b.size()-1)){
                temp1=0;
            }else{
                temp1=b[i-j];
            }
            temp2+=a[j]*temp1;
        }
        result.emplace_back(temp2*t_Div);
    }
}

void conv(std::vector<double>& result,
          const std::vector<double>& a,
          const std::vector<double>& b){
    double temp1,temp2;
    for(size_t i=0;i<a.size()+b.size()-1;i++){
        temp1=0;
        temp2=0;
        for(size_t j=0;j<a.size();j++){
            if(i<j||i-j>(b.size()-1)){
                temp1=0;
            }else{
                temp1=b[i-j];
            }
            temp2+=a[j]*temp1;
        }
        result.emplace_back(temp2);
    }
}
///
enum class Action {
    Up,
    Down,
    Left,
    Right,
    Wait,


};

std::ostream& operator<<(std::ostream& os, const Action& a) {
    switch (a) {
    case Action::Up:
        os << "Up";
        break;
    case Action::Down:
        os << "Down";
        break;
    case Action::Left:
        os << "Left";
        break;
    case Action::Right:
        os << "Right";
        break;
    case Action::Wait:
        os << "Wait";
        break;


    }
    return os;
}

void triangleProbFuc(std::vector<double>& result,
                     double a,
                     double b,
                     double c,
                     double tDiv,
                     double tEnd){
    std::vector<double> t;
    std::vector<int> ind;
    double p=2/(c-a);
    for(double tt=0;tt<=tEnd;tt+=tDiv){
        t.emplace_back(tt);
    }
    for(size_t i=0;i<=t.size()-1;i++){
        if(t[i]>=a&&t[i]<=b){
            result.emplace_back(p/(b-a)*(t[i]-a));
        }
        else if(t[i]>b&&t[i]<=c){
            result.emplace_back((c-t[i])*p/(c-b));
        }
        else{
            result.emplace_back(0);
        }
    }
}

struct CostTable{
    CostTable(size_t agentID,double cost): agentID(agentID),cost(cost){
        step=1;
        waitTime=0;
        triangleProbFuc(pdf, a*cost,cost,c*cost, t_Div,cost*2);
    }


    CostTable(size_t agentID,size_t step,size_t waitTime):agentID(agentID),step(step),waitTime(waitTime){
    }


    CostTable(size_t agentID,size_t step,size_t waitTime,double cost):agentID(agentID),step(step),waitTime(waitTime),cost(cost){
        int tempSign=1;
        std::vector<double> pdfActions,tempPdfActions;
        double wait=static_cast<double>(waitTime)*time_div;
        for(size_t i=waitTime>0?waitTime:0;i<step;i++){
            if(tempSign==1){
                triangleProbFuc(pdfActions, a*cost,cost,c*cost,t_Div,cost*2);
                tempSign=0;
            }else{
                tempPdfActions.clear();
                tempPdfActions.swap(pdfActions);
                pdfActions.clear();
                std::vector<double> pdfAction;
                triangleProbFuc(pdfAction, a*cost,cost,c*cost,t_Div,cost*2);
                convdx(pdfActions,tempPdfActions,pdfAction);
            }

        }
        if(pdfActions.size()!=0){
            for(double i=0;i<=wait;i+=t_Div){
                pdf.emplace_back(0);
            }
            for(size_t i=0;i<=pdfActions.size()-1;i++){
                pdf.emplace_back(pdfActions[i]);
            }
        }else if(pdfActions.size()==0&&std::abs(wait-0)>t_Div){
            for(double i=0;i<=(wait-t_Div);i+=t_Div){
                pdf.emplace_back(0);
            }
            pdf.emplace_back(1/t_Div);
        }else{
            pdf.emplace_back(1/t_Div);
        }
    }

    size_t agentID;
    size_t step=0;
    size_t waitTime=0;
    double cost;
    std::vector<double> pdf;

    bool operator<(const CostTable& other) const {
        return agentID < other.agentID;
    }
    bool operator==(const CostTable& other) const {
        if(agentID==other.agentID&&step==other.step&&waitTime==other.waitTime){
            return true;
        }
        else{
            return false;
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const CostTable& c) {
        return os << "CostTable" << c.agentID<<std::endl;
    }

};

namespace std {
template <>
struct hash<CostTable> {
    size_t operator()(const CostTable& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.agentID);
        boost::hash_combine(seed, s.step);
        boost::hash_combine(seed, s.waitTime);
        return seed;
    }
};
}  // namespace std

struct EdgeCost{
    std::unordered_set<CostTable> costTable;

    friend std::ostream& operator<<(std::ostream& os, const EdgeCost& c) {
        for (const auto& vc : c.costTable) {
            os << vc << std::endl;
        }
        return os;
    }
};


struct State {
    State(Action lateAction,double time, int x, int y, size_t stepNum,size_t waitTime) :lateAction(lateAction),time(time), x(x), y(y),stepNum(stepNum),waitTime(waitTime){
    }
    bool operator==(const State& s) const {
        return std::abs(time-s.time)<=t_Div*0.1 && x == s.x && y == s.y;
    }

    bool equal(const State& s) const { return x == s.x && y == s.y; }

    friend std::ostream& operator<<(std::ostream& os, const State& s) {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y <<","<< s.StepNum<<")";
    }

    Action nowAction;
    Action lateAction;
    double time;
    int x;
    int y;
    size_t stepNum;
    size_t waitTime;

};

namespace std {
template <>
struct hash<State> {
    size_t operator()(const State& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        return seed;
    }
};
}  // namespace std





/*void pdfConvFuc(std::vector<double>& PDFActions,
                double tDiv,
                std::vector<State>& actions){
    double waitTime=0;
    int tempSign=1;
    std::vector<double> pdfActions,tempPdfActions;
    for(size_t i=0;i<=actions.size()-1&&actions.size()>=1;i++ ){
        if(actions[i].nowAction== Action::Wait){
            waitTime+=time_div;
        }
        else{
            if(tempSign==1){
                for(size_t j=0;j<=actions[i].pdf.size()-1;j++){
                    pdfActions.emplace_back(actions[i].pdf[j]);
                }
                tempSign=0;
            }else{
                tempPdfActions.clear();
                tempPdfActions.swap(pdfActions);
                pdfActions.clear();
                conv(pdfActions,tempPdfActions,actions[i].pdf);
                for(size_t j=0;j<=pdfActions.size()-1;j++){
                    pdfActions[j]=pdfActions[j]*tDiv;
                }

            }
        }
    }
    if(pdfActions.size()!=0){
        for(double i=0;i<=waitTime;i+=tDiv){
            PDFActions.emplace_back(0);
        }
        for(size_t i=0;i<=pdfActions.size()-1;i++){
            PDFActions.emplace_back(pdfActions[i]);
        }
    }else if(pdfActions.size()==0&&std::abs(waitTime-0)>=0.0001){
        for(double i=0;i<=(waitTime-tDiv);i+=tDiv){
            PDFActions.emplace_back(0);
        }
        PDFActions.emplace_back(1/tDiv);
    }else{
        PDFActions.emplace_back(0);
    }

}*/


void probSum(double& probResult,
             const std::vector<double>& pdfActions,
             double boundL,
             double boundH,
             double tDiv){
    probResult=0;
    for(size_t i=0;i<=pdfActions.size()-1;i++){
        if (tDiv*i>boundL&&tDiv*i<=boundH){
            probResult+=tDiv*pdfActions[i];
        }else if(tDiv*i>boundH){
            break;
        }
    }
}

void edgeProbCal(double& prob,
                 const std::vector<double>& pdfActions1a,
                 const std::vector<double>& pdfActions1b,
                 const std::vector<double>& pdfActions2a,
                 const std::vector<double>& pdfActions2b,
                 double tDiv){
    prob=0;
    double regionL=0;
    double regionH=(pdfActions2a.size()-1)*tDiv;
    for(size_t i=0;i<=pdfActions2a.size()-1;i++){
        if(pdfActions2a[i]>tDiv*0.1){
            regionL=i*tDiv;
            break;
        }
    }

    for(size_t i=pdfActions2a.size()-1;i!=0;i--){
        if(pdfActions2a[i]>tDiv*0.1){
            regionH=i*tDiv;
            break;
        }
    }

    for(size_t i=0;i<=pdfActions1b.size()-1;i++){
        double boundL=tDiv*i;
        double boundH=(pdfActions2a.size()-1)*tDiv;
        if(regionH<boundL){
            continue;
        }else{
            double temp;
            probSum(temp,pdfActions2a,boundL,boundH,tDiv);
            prob+=temp*tDiv*pdfActions1b[i];
        }
    }

    for(size_t i=0;i<=pdfActions2b.size()-1;i++){
        if(pdfActions2a[i]>tDiv*0.1){
            regionL=i*tDiv;
            break;
        }
    }

    for(size_t i=pdfActions2b.size()-1;i!=0;i--){
        if(pdfActions2a[i]>tDiv*0.1){
            regionH=i*tDiv;
            break;
        }
    }

    for(size_t i=0;i<=pdfActions1a.size()-1;i++){
        double boundL=0;
        double boundH=tDiv*i;
        if(boundH<regionL){
            continue;
        }else{
            double temp;
            probSum(temp,pdfActions2b,boundL,boundH,tDiv);
            prob+=temp*tDiv*pdfActions1a[i];
        }
    }
    prob=1-prob;
}

void vertexProbCal(double& prob,
                   const std::vector<double>& pdfActions1,
                   const std::vector<double>& pdfActions2,
                   double tDiv,
                   double thresholdCost){
    prob=0;
    double regionL=0;
    double regionH=(pdfActions2.size()-1)*tDiv;
    for(size_t i=0;i<=pdfActions2.size()-1;i++){
        if(pdfActions2[i]>tDiv*0.1){
            regionL=i*tDiv;
            break;
        }
    }

    for(size_t i=pdfActions2.size()-1;i!=0;i--){
        if(pdfActions2[i]>tDiv*0.1){
            regionH=i*tDiv;
            break;
        }
    }

    for(size_t i=0;i<=pdfActions1.size()-1;i++){
        double boundL=tDiv*i-thresholdCost;
        double boundH=tDiv*i+thresholdCost;
        if(boundH<regionL||regionH<boundL){
            continue;
        }else{
            double temp;
            probSum(temp,pdfActions2,boundL,boundH,tDiv);
            prob+=temp*tDiv*pdfActions1[i];
        }
    }
}


State getStateByTime(size_t agentIdx,
                     const std::vector<PlanResult<State, Action, double> >& solution,
                     double time) {

    assert(agentIdx < solution.size());

    if (time < solution[agentIdx].states.back().second) {
        size_t tempIdex;
        for(tempIdex=solution[agentIdx].states.size()-1;;tempIdex--)
        {
            if(time>=solution[agentIdx].states[tempIdex].first.time)
            {
                break;
            }
        }
        return solution[agentIdx].states[tempIdex].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
}

struct Conflict {
    enum Type {
        Vertex,
        Edge,
        Vertex_end1,
        Vertex_end2

    };


    double agent1Time;
    double agent2Time;
    double agent1NextTime;
    double agent2NextTime;
    size_t agent1;
    size_t agent2;
    Type type;

    int x1;
    int y1;
    int x2;
    int y2;
    int x3;
    int y3;
    friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
        switch (c.type) {
        case Vertex:
            return os << c.agent1Time<<","<<c.agent2Time<< ": Vertex(" << c.x1 << "," << c.y1 << ")";
        case Edge:
            return os << c.agent1Time<<","<<c.agent1NextTime<<","<<c.agent1NextTime<<","<<c.agent2Time<< ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                      << "," << c.y2 << ","<<c.x3<<","<<c.y3<<")";
        case Vertex_end1:
            return os << c.agent1Time<<","<<c.agent2Time<< ": Vertex(" << c.x1 << "," << c.y1 << ")";
        case Vertex_end2:
            return os << c.agent1Time<<","<<c.agent2Time<< ": Vertex(" << c.x1 << "," << c.y1 << ")";
        }
        return os;
    }
};

struct VertexConstraint {
    VertexConstraint(double lowTime,double highTime, int x, int y) : lowTime(lowTime),highTime(highTime), x(x), y(y) {}
    double lowTime;
    double highTime;
    int x;
    int y;

    bool operator<(const VertexConstraint& other) const {
        return std::tie(highTime, x, y) < std::tie(other.highTime, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const {
        if(x==other.x&&y==other.y)
        {
            if(lowTime<=other.highTime&&lowTime>=other.lowTime){
                return true;
            }
            else {
                return false;
            }
        }else{
            return false;
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
        return os << "VC(" << c.lowTime<<","<<c.highTime << "," << c.x << "," << c.y << ")";
    }
};

namespace std {
template <>
struct hash<VertexConstraint> {
    size_t operator()(const VertexConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        return seed;
    }
};
}  // namespace std

struct EdgeConstraint {
    EdgeConstraint(double lowTime,double highTime, int x1, int y1, int x2, int y2)
        : lowTime(lowTime),highTime(highTime), x1(x1), y1(y1), x2(x2), y2(y2) {}
    double lowTime;
    double highTime;
    int x1;
    int y1;
    int x2;
    int y2;

    bool operator<(const EdgeConstraint& other) const {
        return std::tie(highTime, x1, y1, x2, y2) <
                std::tie(other.highTime, other.x1, other.y1, other.x2, other.y2);
    }

    /*bool operator==(const EdgeConstraint& other) const {
        return std::tie(lowTime, highTime,x1, y1, x2, y2) ==
               std::tie(other.lowTime,other.highTime, other.x1, other.y1, other.x2, other.y2);
      }*/

    bool operator==(const EdgeConstraint& other) const {
        if(x1==other.x1&&y1==other.y1&&x2==other.x2&&y2==other.y2)
        {
            if((lowTime<=other.highTime&&lowTime>=other.lowTime)||(highTime>=other.lowTime&&highTime<=other.highTime)){
                return true;
            }
            else {
                return false;
            }
        }else{
            return false;
        }
    }
    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
        return os << "EC(" << c.lowTime<<","<<c.highTime << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
    size_t operator()(const EdgeConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.x1);
        boost::hash_combine(seed, s.y1);
        boost::hash_combine(seed, s.x2);
        boost::hash_combine(seed, s.y2);
        return seed;
    }
};
}  // namespace std

struct Constraints {
    std::unordered_multiset<VertexConstraint> vertexConstraints;
    std::unordered_multiset<EdgeConstraint> edgeConstraints;
    void add(const Constraints& other) {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const Constraints& other) {
        std::vector<VertexConstraint> vertexIntersection;
        std::vector<EdgeConstraint> edgeIntersection;
        std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                              other.vertexConstraints.begin(),
                              other.vertexConstraints.end(),
                              std::back_inserter(vertexIntersection));
        std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                              other.edgeConstraints.begin(),
                              other.edgeConstraints.end(),
                              std::back_inserter(edgeIntersection));
        return !vertexIntersection.empty() || !edgeIntersection.empty();
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
        for (const auto& vc : c.vertexConstraints) {
            os << vc << std::endl;
        }
        for (const auto& ec : c.edgeConstraints) {
            os << ec << std::endl;
        }
        return os;
    }
};




struct Location {
    Location(int x, int y) : x(x), y(y) {}
    int x;
    int y;

    bool operator<(const Location& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Location& other) const {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const Location& c) {
        return os << "(" << c.x << "," << c.y << ")";
    }
};

namespace std {
template <>
struct hash<Location> {
    size_t operator()(const Location& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        return seed;
    }
};
}  // namespace std

///
class Environment {
public:
    Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                std::vector<Location> goals)
        : m_dimx(dimx),
          m_dimy(dimy),
          m_obstacles(std::move(obstacles)),
          m_goals(std::move(goals)),
          m_agentIdx(0),
          m_constraints(nullptr),
          m_lastGoalConstraint(-1),
          m_highLevelExpanded(0),
          m_lowLevelExpanded(0) {
        // computeHeuristic();
    }

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
        assert(constraints);  // NOLINT
        m_agentIdx = agentIdx;
        m_constraints = constraints;
        m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints) {
            if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
                m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.highTime);
            }
        }
    }

    int admissibleHeuristic(const State& s) {
       // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
       // s.y] << std::endl;
       // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
       return std::abs(s.x - m_goals[m_agentIdx].x) +
              std::abs(s.y - m_goals[m_agentIdx].y);
     }


    bool isSolution(const State& s) {
        return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
                s.time > m_lastGoalConstraint;
    }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, double> >& neighbors,
                      EdgeCost& edgeCost) {
        // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
        // for(const auto& vc : constraints.vertexConstraints) {
        //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   std::endl;
        // }
        auto sigleAction=edgeCost.costTable.find(CostTable(m_agentIdx,1,0));
        neighbors.clear();
        {
            State n(Action::Wait,s.time + time_div, s.x, s.y,s.stepNum+1,s.waitTime+1);
            if (stateValid(n) && transitionValid(s, n)) {neighbors.emplace_back(Neighbor<State, Action, double>(n, Action::Wait, time_div));}
        }
        {
            State n(Action::Left,s.time + sigleAction->cost, s.x - 1, s.y,s.stepNum+1,s.waitTime);
            if (stateValid(n) && transitionValid(s, n)) {neighbors.emplace_back(Neighbor<State, Action, double>(n, Action::Left, sigleAction->cost));}
        }
        {
            State n(Action::Right,s.time + sigleAction->cost, s.x + 1, s.y,s.stepNum+1,s.waitTime);
            if (stateValid(n) && transitionValid(s, n)) {neighbors.emplace_back(Neighbor<State, Action, double>(n, Action::Right, sigleAction->cost));}
        }
        {
            State n(Action::Up,s.time+ sigleAction->cost, s.x, s.y + 1,s.stepNum+1,s.waitTime);
            if (stateValid(n) && transitionValid(s, n)) {neighbors.emplace_back(Neighbor<State, Action, double>(n, Action::Up, sigleAction->cost));}
        }
        {
            State n(Action::Down,s.time + sigleAction->cost, s.x, s.y - 1,s.stepNum+1,s.waitTime);
            if (stateValid(n) && transitionValid(s, n)) { neighbors.emplace_back(Neighbor<State, Action, double>(n, Action::Down, sigleAction->cost));}
        }


    }


    bool getFirstConflict(
            const std::vector<PlanResult<State, Action, double> >& solution,
            Conflict& result,
            EdgeCost& edgeCost) {
        double tDiv=t_Div;
        double thresholdCost=threshold;

        // check drive-drive vertex collisions
        for (size_t i = 0; i < solution.size(); ++i) {
            for (size_t j = i + 1; j < solution.size(); ++j) {
                for(size_t stepNum1 = 1; stepNum1 <=solution[i].states.size() - 1;stepNum1++)
                {
                    for(size_t stepNum2 =1; stepNum2 <= solution[j].states.size() - 1;stepNum2++)
                    {
                        State state1 = getState(i, solution, stepNum1);
                        State state2 = getState(j, solution, stepNum2);

                        if (state1.equal(state2)) {
                            if (state1.stepNum==(solution[i].states.size() - 1)&&state1.time<=state2.time){
                                result.agent1Time =state1.time-thresholdCost;
                                result.agent2Time =state2.time+thresholdCost;
                                result.agent1 = i;
                                result.agent2 = j;
                                result.type = Conflict::Vertex_end1;
                                result.x1 = state1.x;
                                result.y1 = state1.y;
                                std::cout << "VC1 " <<"agent"<<i<<","<<"agent"<<j<<"at"<< "(" << state1.x << "," << state1.y <<")"<<
                                             std::endl;
                                return true;
                            }
                            if (state2.stepNum==(solution[j].states.size() - 1)&&state2.time<=state1.time){
                                result.agent1Time =state1.time+thresholdCost;
                                result.agent2Time =state2.time-thresholdCost;
                                result.agent1 = i;
                                result.agent2 = j;
                                result.type = Conflict::Vertex_end2;
                                result.x1 = state2.x;
                                result.y1 = state2.y;
                                std::cout << "VC2 " <<"agent"<<i<<","<<"agent"<<j<<"at"<< "(" << state1.x << "," << state1.y <<")"<<
                                             std::endl;
                                return true;
                            }
                            /*if(std::abs(state1.time-state2.time)>10*thresholdCost) continue;
                            double prob=0;
                            auto s1pdf=edgeCost.costTable.find(CostTable(i,state1.stepNum,state1.waitTime));
                            auto s2pdf=edgeCost.costTable.find(CostTable(j,state2.stepNum,state2.waitTime));
                            vertexProbCal(prob,s1pdf->pdf,s2pdf->pdf,tDiv,thresholdCost);*/
                            if(std::abs(state1.time-state2.time)<=thresholdCost){
                                result.agent1Time =min(state1.time,state2.time)-thresholdCost;
                                result.agent2Time =max(state1.time,state2.time)+thresholdCost;
                                result.agent1 = i;
                                result.agent2 = j;
                                result.type = Conflict::Vertex;
                                result.x1 = state1.x;
                                result.y1 = state1.y;
                                std::cout << "VC3 " <<"agent"<<i<<","<<"agent"<<j<<"at"<< "(" << state1.x << "," << state1.y <<")"<<std::endl;
                                return true;
                            }
                        }

                    }
                }
            }
        }

        // drive-drive edge (swap)
        for (size_t i = 0; i < solution.size(); ++i) {
            for (size_t j = i + 1; j < solution.size(); ++j) {
                for(size_t stepNum1 = 0; stepNum1 < solution[i].states.size() - 1;stepNum1++)
                {
                    for(size_t stepNum2 = 0; stepNum2 < solution[j].states.size() - 1;stepNum2++)
                    {
                        State state1a = getState(i, solution, stepNum1);
                        State state2a = getState(j, solution, stepNum2);
                        State state1b = getState(i, solution, stepNum1+1);
                        State state2b = getState(j, solution, stepNum2+1);
                        if (state1a.equal(state2b)&&
                                state1b.equal(state2a)){
                            /*if(state1a.time-state2b.time>3||state2a.time-state1b.time>3) continue;
                            double prob=0;
                            auto s1apdf=edgeCost.costTable.find(CostTable(i,state1a.stepNum,state1a.waitTime));
                            auto s1bpdf=edgeCost.costTable.find(CostTable(i,state1b.stepNum,state1b.waitTime));
                            auto s2apdf=edgeCost.costTable.find(CostTable(j,state2a.stepNum,state2a.waitTime));
                            auto s2bpdf=edgeCost.costTable.find(CostTable(j,state2b.stepNum,state2b.waitTime));
                            edgeProbCal(prob,s1apdf->pdf,s1bpdf->pdf,s2apdf->pdf,s2bpdf->pdf,tDiv);
                            if(stepNum1==0&&stepNum2==0){prob=1;};
                            printf("prob:%lf,agent%d,agent%d,at(%d,%d)\n",prob,i,j,stepNum1,stepNum2);*/
                            if((state1a.time<=state2b.time&&state1a.time>=state2a.time)||(state2a.time>=state1a.time&&state2a.time<=state1b.time)){
                                result.agent1Time =state1a.time;
                                result.agent1NextTime=state1b.time;
                                result.agent2Time =state2a.time;
                                result.agent2NextTime=state2b.time;
                                result.agent1 = i;
                                result.agent2 = j;
                                result.type = Conflict::Edge;
                                result.x1 = state1a.x;
                                result.y1 = state1a.y;
                                result.x2 = state1b.x;
                                result.y2 = state1b.y;
                                result.x3 = state1b.x;
                                result.y3 = state1b.y;
                                std::cout << "EC1 " <<"agent"<<i<<","<<"agent"<<j<<"at"<< "(" << state1a.x << "," << state1a.y <<")"<<"(" << state1b.x << "," << state1b.y <<")"<<
                                             std::endl;
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    void createConstraintsFromConflict(const Conflict& conflict,
                                       std::map<size_t, Constraints>& constraints,
                                       const std::vector<PlanResult<State, Action, double> >& solution) {

        if (conflict.type == Conflict::Vertex) {
            Constraints c1;
            //double lowTime,highTime;
            //lowTime=min(conflict.agent1Time,conflict.agent2Time);
            //highTime=max(conflict.agent1Time,conflict.agent2Time);
            c1.vertexConstraints.emplace(
                        VertexConstraint(conflict.agent1Time,conflict.agent2Time, conflict.x1, conflict.y1));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        }else if(conflict.type == Conflict::Vertex_end1){
            Constraints c1;
            double lowTime,highTime;
            lowTime=conflict.agent1Time;
            highTime=conflict.agent2Time;
            c1.vertexConstraints.emplace(
                        VertexConstraint(lowTime,highTime, conflict.x1, conflict.y1));
            constraints[conflict.agent1] = c1;
        } else if(conflict.type == Conflict::Vertex_end2){
            Constraints c1;
            double lowTime,highTime;
            lowTime=conflict.agent2Time;
            highTime=conflict.agent1Time;
            c1.vertexConstraints.emplace(
                        VertexConstraint(lowTime,highTime, conflict.x1, conflict.y1));
            constraints[conflict.agent2] = c1;
        }
        else if (conflict.type == Conflict::Edge) {
            Constraints c1,c2;
            if(conflict.x2==conflict.x3&&conflict.y2==conflict.y3){
                c1.edgeConstraints.emplace(EdgeConstraint(
                                               conflict.agent1Time,conflict.agent1NextTime, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
                c2.edgeConstraints.emplace(EdgeConstraint(
                                               conflict.agent2Time,conflict.agent2NextTime, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
                constraints[conflict.agent1] = c1;
                constraints[conflict.agent2] = c2;
            }
            else{
                c1.edgeConstraints.emplace(EdgeConstraint(
                                               conflict.agent1Time,conflict.agent1NextTime, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
                c2.edgeConstraints.emplace(EdgeConstraint(
                                               conflict.agent2Time,conflict.agent2NextTime, conflict.x1, conflict.y1, conflict.x3, conflict.y3));
                constraints[conflict.agent1] = c1;
                constraints[conflict.agent2] = c2;
            }


        }
    }

    void onExpandHighLevelNode(double /*cost*/) { m_highLevelExpanded++; }

    void onExpandLowLevelNode(const State& /*s*/, double /*fScore*/,
                              double /*gScore*/) {
        m_lowLevelExpanded++;
    }

    int highLevelExpanded() { return m_highLevelExpanded; }

    int lowLevelExpanded() const { return m_lowLevelExpanded; }


private:
    State getState(size_t agentIdx,
                   const std::vector<PlanResult<State, Action, double> >& solution,
                   size_t stepNum) {

        assert(agentIdx < solution.size());

        if (stepNum < solution[agentIdx].states.size()) {
            return solution[agentIdx].states[stepNum].first;
        }
        assert(!solution[agentIdx].states.empty());
        return solution[agentIdx].states.back().first;
    }



    /*State getNextState(size_t agentIdx,
                       const std::vector<PlanResult<State, Action, int> >& solution,
                       int t) {

        assert(agentIdx < solution.size());

        if (t < solution[agentIdx].cost) {
            size_t tempIdex;
            for(tempIdex=solution[agentIdx].states.size()-2;tempIdex>=0;tempIdex--)
            {
                if(t>=solution[agentIdx].states[tempIdex].second)
                {
                    break;
                }
            }
            return solution[agentIdx].states[tempIdex+1].first;
        }
        assert(!solution[agentIdx].states.empty());
        return solution[agentIdx].states.back().first;
    }*/

    bool stateValid(const State& s) {
        assert(m_constraints);
        const auto& con = m_constraints->vertexConstraints;
        return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
                m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() && con.find(VertexConstraint(s.time,s.time, s.x, s.y)) == con.end();
    }

    bool transitionValid(const State& s1, const State& s2) {
        assert(m_constraints);
        const auto& con = m_constraints->edgeConstraints;
        return con.find(EdgeConstraint(s1.time,s2.time, s1.x, s1.y, s2.x, s2.y)) ==con.end();
    }
#if 0
    // We use another A* search for simplicity
    // we compute the shortest path to each goal by using the fact that our getNeighbor function is
    // symmetric and by not terminating the AStar search until the queue is empty
    void computeHeuristic()
    {
        class HeuristicEnvironment
        {
        public:
            HeuristicEnvironment(
                        size_t dimx,
                        size_t dimy,
                        const std::unordered_set<Location>& obstacles,
                        std::vector<int>* heuristic)
                : m_dimx(dimx)
                , m_dimy(dimy)
                , m_obstacles(obstacles)
                , m_heuristic(heuristic)
            {
            }

            int admissibleHeuristic(
                        const Location& s)
            {
                return 0;
            }

            bool isSolution(
                        const Location& s)
            {
                return false;
            }

            void getNeighbors(
                        const Location& s,
                        std::vector<Neighbor<Location, Action, int> >& neighbors)
            {
                neighbors.clear();

                {
                    Location n(s.x-1, s.y);
                    if (stateValid(n)) {
                        neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Left, 1));
                    }
                }
                {
                    Location n(s.x+1, s.y);
                    if (stateValid(n)) {
                        neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Right, 1));
                    }
                }
                {
                    Location n(s.x, s.y+1);
                    if (stateValid(n)) {
                        neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Up, 1));
                    }
                }
                {
                    Location n(s.x, s.y-1);
                    if (stateValid(n)) {
                        neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Down, 1));
                    }
                }
            }

            void onExpandNode(
                        const Location& s,
                        int fScore,
                        int gScore)
            {
            }

            void onDiscover(
                        const Location& s,
                        int fScore,
                        int gScore)
            {
                (*m_heuristic)[s.x + m_dimx * s.y] = gScore;
            }

        private:
            bool stateValid(
                        const Location& s)
            {
                return    s.x >= 0
                        && s.x < m_dimx
                        && s.y >= 0
                        && s.y < m_dimy
                        && m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
            }

        private:
            int m_dimx;
            int m_dimy;
            const std::unordered_set<Location>& m_obstacles;
            std::vector<int>* m_heuristic;

        };

        m_heuristic.resize(m_goals.size());

        std::vector< Neighbor<State, Action, int> > neighbors;

        for (size_t i = 0; i < m_goals.size(); ++i) {
            m_heuristic[i].assign(m_dimx * m_dimy, std::numeric_limits<int>::max());
            HeuristicEnvironment henv(m_dimx, m_dimy, m_obstacles, &m_heuristic[i]);
            AStar<Location, Action, int, HeuristicEnvironment> astar(henv);
            PlanResult<Location, Action, int> dummy;
            astar.search(m_goals[i], dummy);
            m_heuristic[i][m_goals[i].x + m_dimx * m_goals[i].y] = 0;
        }
    }
#endif
private:
    int m_dimx;
    int m_dimy;
    std::unordered_set<Location> m_obstacles;
    std::vector<Location> m_goals;
    // std::vector< std::vector<int> > m_heuristic;
    size_t m_agentIdx;
    const Constraints* m_constraints;
    // const EdgeCost m_edgeCost;
    double m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
};

int main(int argc, char* argv[]) {
    /* Constraints a;
    a.edgeConstraints.emplace(EdgeConstraint(2,4,1,1,1,1));
    auto b=a.edgeConstraints.count(EdgeConstraint(0,3,1,1,1,1));
    a.vertexConstraints.emplace(VertexConstraint(0,2,1,1));
    auto c=a.vertexConstraints.count(VertexConstraint(1,1.5,1,1));
    return 1;*/
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    desc.add_options()("help", "produce help message")(
                "input,i", po::value<std::string>(&inputFile)->required(),
                "input file (YAML)")("output,o",
                                     po::value<std::string>(&outputFile)->required(),
                                     "output file (YAML)");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u) {
            std::cout << desc << "\n";
            return 0;
        }
    } catch (po::error& e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    YAML::Node config = YAML::LoadFile(inputFile);

    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    std::vector<State> startStates;

    const auto& dim = config["map"]["dimensions"];
    int dimx = dim[0].as<int>();
    int dimy = dim[1].as<int>();

    for (const auto& node : config["map"]["obstacles"]) {
        obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }

    for (const auto& node : config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        startStates.emplace_back(State(Action::Wait,0, start[0].as<int>(), start[1].as<int>(),0,0));
        // std::cout << "s: " << startStates.back() << std::endl;
        goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    }

    Environment mapf(dimx, dimy, obstacles, goals);
    CBS<State, Action,EdgeCost,CostTable, double, Conflict, Constraints, Environment> cbs(mapf);
    std::vector<PlanResult<State, Action, double> > solution;
    EdgeCost edgeCost;
    for(size_t agentID=0;agentID<startStates.size();agentID++){
        double randomCost=velocity;//+static_cast<double>(agentID+1)*2/10;
        edgeCost.costTable.insert(CostTable(agentID,randomCost));
        edgeCost.costTable.insert(CostTable(agentID,0,0,randomCost));
    }
    /*for(size_t agentID=0;agentID<startStates.size();agentID++){
        for(int i=0;i<dimx;i++){
            for(int j=0;j<dimy;j++){
                if(i!=dimx-1){
                    //double randomCost=2;
                    double randomCost=1+static_cast<double>(rand())*2/RAND_MAX;
                    edgeCost.costTable.insert(CostTable(agentID,i,j,i+1,j,randomCost));
                    edgeCost.costTable.insert(CostTable(agentID,i+1,j,i,j,randomCost));

                }
                if(j!=dimy-1){
                    //double randomCost=2;
                    double randomCost=1+static_cast<double>(rand())*2/RAND_MAX;
                    edgeCost.costTable.insert(CostTable(agentID,i,j,i,j+1,randomCost));
                    edgeCost.costTable.insert(CostTable(agentID,i,j+1,i,j,randomCost));
                }
            }
        }
    }*/

    Timer timer;
    bool success = cbs.search(startStates, solution, edgeCost);
    timer.stop();

    if (success) {
        std::cout << "Planning successful! " << std::endl;
        double cost = 0;
        double makespan = 0;
        for (const auto& s : solution) {
            cost += s.cost;
            makespan = std::max<double>(makespan, s.cost);
        }
        std::cout<<"complete"<<std::endl;

        /*for(size_t i=0;i<solution.size();i++){
            for(size_t j=1;j<=solution[i].states.size()-1;j++){
                switch(solution[i].states[j-1].first.nowAction){
                case Action::Wait:
                    solution[i].states[j].second=solution[i].states[j-1].second+time_div;
                    break;
                default:
                    solution[i].states[j].second=solution[i].states[j-1].second+generateRandomTime(solution[i].states[j-1].first.pdf,t_Div);
                    break;
                }
            }
        }*/
        std::ofstream out(outputFile);
        out << "statistics:" << std::endl;
        out << "  cost: " << cost << std::endl;
        out << "  makespan: " << makespan << std::endl;
        out << "  runtime: " << timer.elapsedSeconds() << std::endl;
        out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
        out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
        out << "schedule:" << std::endl;
        for (size_t a = 0; a < solution.size(); ++a) {
            // std::cout << "Solution for: " << a << std::endl;
            // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
            //   std::cout << solution[a].states[i].second << ": " <<
            //   solution[a].states[i].first << "->" << solution[a].actions[i].first
            //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
            // }
            // std::cout << solution[a].states.back().second << ": " <<
            // solution[a].states.back().first << std::endl;

            out << "  agent" << a << ":" << std::endl;
            for (const auto& state : solution[a].states) {
                out << "    - x: " << state.first.x << std::endl
                    << "      y: " << state.first.y << std::endl
                    << "      t: " << state.first.time<< std::endl;
            }
        }
    } else {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}
