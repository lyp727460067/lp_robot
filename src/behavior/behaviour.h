#ifndef _BEHAVIOUER_H
#define  _BEHAVIOUER_H
#include <string>
#include <vector>
#include <list>
#include <memory>
enum class BehaviourState {
  Completed,
  Failure,
  Running,
};
class BaseNode {
 public:
  virtual std::string Name() = 0;
  virtual BehaviourState State() = 0;
  virtual bool Enter() = 0;
  virtual bool Tick() = 0;
  virtual bool Exit() = 0;
};
class  CompositeNode:public BaseNode {
 public:
  virtual std::string Name() = 0;
  virtual BehaviourState State() = 0;
  virtual bool Enter() = 0;
  virtual bool Tick() = 0;
  virtual bool Exit() = 0;
  void AddNode(BaseNode*node );
  bool HasNode();

  void AddCondition();
  void RemoveCondition();
  bool HasCondition();

 protected:
  
  std::list<BaseNode*> node_list_;
  std::list<BaseNode*> condition_list_; 
};
class SelectorNode :public  CompositeNode{
 public:
  virtual std::string Name() = 0;
  virtual BehaviourState State() = 0;
  virtual bool Enter() = 0;
  virtual bool Tick() = 0;
  virtual bool Exit() = 0;
};
class   SequenceNode:public  CompositeNode{
 public:
  virtual std::string Name() = 0;
  virtual BehaviourState State() = 0;
  virtual bool Enter() = 0;
  virtual bool Tick() = 0;
  virtual bool Exit() = 0;
};
class   ParallelNode:public  CompositeNode{
 public:
  virtual std::string Name() = 0;
  virtual BehaviourState State() = 0;
  virtual bool Enter() = 0;
  virtual bool Tick() = 0;
  virtual bool Exit() = 0;
};
class   DecoratorNode:public BaseNode{
 public:
  virtual std::string Name() = 0;
  virtual BehaviourState State() = 0;
  virtual bool Enter() = 0;
  virtual bool Excute() = 0;
  virtual bool Exit() = 0;
};
//
class CoditionNode :public BaseNode{

};

class ActionNode :public BaseNode
{
};
class BaseActionNode :public ActionNode
{


};
class BehaviourTree
{


};


#endif