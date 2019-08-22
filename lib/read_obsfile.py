#!/usr/bin/env python

def read(path):
    obs_items = open(path, 'r').read().split('\n')
    obs = {}
    for _item in obs_items:
        if _item.startswith('script;'): break
        _item = _item.split('#')[0]
        _key, _value = _item.split('=', 1)
        _key = _key.strip()
        _value = _value.strip()
        try:
            obs[_key] = eval(_value)
        except:
            try:
                obs[_key] = obs[_value]
            except:
                pass
        if not _value.find('/') == -1 and _value.find('*') == -1:
            _value1,_value2 = _value.split('/',1)
            _value1 = _value1.strip()
            _value2 = _value2.strip()
            if isinstance(_value1,str):
                try:
                    _value1 = float(_value1)
                except:
                    pass
            else:
                pass
            if isinstance(_value2,str):
                try:
                    _value2 = float(_value2)
                except:
                    pass
            else:
                pass
            if isinstance(_value1,float) and isinstance(_value2,float):
                obs[_key] = (float(_value1)*100)/(float(_value2)*100)
            elif not isinstance(_value1,float) and not isinstance(_value2,float):
                obs[_key] = (obs[_value1]*100)/(obs[_value2]*100)
            elif isinstance(_value1,float):
                obs[_key] = (float(_value1)*100)/(obs[_value2]*100)
            elif isinstance(_value2,float):
                obs[_key] = (obs[_value1]*100)/(float(_value2)*100)
            else:
                print('Error')
        elif _value.find('/') == -1 and not _value.find('*') == -1:
            _value1,_value2 = _value.split('*',1)
            _value1 = _value1.strip()
            _value2 = _value2.strip()
            if isinstance(_value1,str):
                try:
                    _value1 = float(_value1)
                except:
                    pass
            if isinstance(_value2,str):
                try:
                    _value2 = float(_value2)
                except:
                    pass
            if isinstance(_value1,float) and isinstance(_value2,float):
                obs[_key] = (float(_value1)*100)*(float(_value2)*100)/10000
            elif not isinstance(_value1,float) and not isinstance(_value2,float):
                obs[_key] = (obs[_value1]*100)*(obs[_value2]*100)/10000
            elif isinstance(_value1,float):
                obs[_key] = (float(_value1)*100)*(obs[_value2]*100)/10000
            elif isinstance(_value2,float):
                obs[_key] = (obs[_value1]*100)*(float(_value2)*100)/10000
            else:
                print('Error')
        else:
            pass
    return obs