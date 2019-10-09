#ifndef BEHAVIOUR_MAP_H 
#define BEHAVIOUR_MAP_H

#include <vector>
#include<string>
namespace Behaviours
{

class Behaviour
{
    public:
        Behaviour(std::string beh,std::string desc) :behaviour(beh),description(desc){}
        std::string behaviour;
        std::string description;
};

//TODO use c++11 for static initialization
std::vector<Behaviour> create_beh_list()
{
    std::vector<Behaviour> behL;
    behL.push_back( Behaviour("macarena-ef2c97/behavior_1","macarena!") );
    behL.push_back( Behaviour("tracking-d31feb/behavior_1","Ψάχνω το μπαλάκι παιδιά!") );
    behL.push_back( Behaviour("danceevolution-faf871/behavior_1","Ώρα για Thriller!") );   
    behL.push_back( Behaviour("gangnam-57eec6/behavior_1","gangnam Style!") );    
    behL.push_back( Behaviour("vangelis-6f4014/behavior_1","Πασόκ Σώσε μας Σε παρακαλώ!") );
    behL.push_back( Behaviour("koyrastika-91e979/behavior_1","") );
    behL.push_back( Behaviour("warmhello-4f850e/behavior_1","") );
    behL.push_back( Behaviour("taichi-6066a5/behavior_1","Είμαι το Κung-Fu Πάντα") );
    behL.push_back( Behaviour("eyeofthetiger-84c49a/behavior_1","Είμαι ο Rocky!") );
    
    
    return behL;
    
};
    
const std::vector<Behaviour> behaviourL =create_beh_list();

}
#endif
