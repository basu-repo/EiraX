import ipaddress,json,sys
import pandas as pd

df=pd.read_csv(sys.argv[1]); 
s=json.load(open(sys.argv[2])); 
e=[]

if list(df.columns)!=s['columns']:e.append('column names/order mismatch')

for c,a in s['categorical_values'].items():
 b=set(df[c].dropna().astype(str))-set(a)
 if b:e.append(f'{c}: invalid {sorted(b)}')

for c in s['integer_nonnegative']:
 v=pd.to_numeric(df[c],errors='coerce')
 if v.isna().any() or (v<0).any() or ((v%1)!=0).any():e.append(f'{c}: invalid integer')

for c in s['number_nonnegative']:
 v=pd.to_numeric(df[c],errors='coerce')
 if v.isna().any() or (v<0).any():e.append(f'{c}: invalid number')

for c in s['unit_interval']:
 v=pd.to_numeric(df[c],errors='coerce')
 if v.isna().any() or not v.between(0,1).all():e.append(f'{c}: outside [0,1]')

for c in s['binary']:
 v=pd.to_numeric(df[c],errors='coerce')
 if v.isna().any() or not set(v.unique()).issubset({0,1}):e.append(f'{c}: not binary')

for c in s['ports']:
 if not pd.to_numeric(df[c],errors='coerce').between(0,65535).all():e.append(f'{c}: bad port')

for c in s['ip_addresses']:
 for x in df[c].astype(str).unique():
  try:ipaddress.ip_address(x)
  except ValueError:e.append(f'{c}: bad IP {x}')

q=s['signal_quality_dbm']
if not pd.to_numeric(df.signal_quality_dbm,errors='coerce').between(q['minimum'],q['maximum']).all():e.append('signal quality out of range')
for a,r in s['response_by_attack'].items():
 if not (df.loc[df.attack_type==a,'recommended_response']==r).all():e.append(f'{a}: response mismatch')

if e:print('RESULT: FAILED\n'+'\n'.join(e));raise SystemExit(1)

print(f'RESULT: PASSED ({len(df)} rows, {len(df.columns)} columns)')
