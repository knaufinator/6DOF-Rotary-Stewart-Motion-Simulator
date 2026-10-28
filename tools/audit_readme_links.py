"""Check every repository README's Markdown/HTML links, images and local anchors.

Run with --external for live HTTP checks. Vendor/submodule results are reported
separately; no documents are rewritten. Requires mistune 3.x (requirements-docs.txt).
"""
from pathlib import Path
from urllib.parse import urlsplit, unquote, urljoin, quote
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError
from concurrent.futures import ThreadPoolExecutor
import argparse, subprocess, json, re, html, datetime
import mistune

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--external',action='store_true')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
markdown=mistune.create_markdown(renderer='ast')
def plain(tokens):
    return ''.join(t.get('raw','')+plain(t.get('children',[])) for t in tokens)
def anchors(p):
    text=p.read_text(encoding='utf-8-sig',errors='replace'); counts={}; result=set()
    for t in markdown(text):
        if t['type']=='heading':
            s=plain(t.get('children',[])).lower().strip()
            s=re.sub(r'<[^>]+>','',s)
            s=re.sub(r'[^\w\- ]','',s).replace(' ','-')
            n=counts.get(s,0); counts[s]=n+1
            result.add(s+('-'+str(n) if n else ''))
    result.update(re.findall(r'(?:id|name)=["\']([^"\']+)',text))
    return result
def links(tokens):
    for t in tokens:
        if t['type'] in ['link','image']:
            yield t['type'],t.get('attrs',{}).get('url','')
        if t['type'] in ['inline_html','block_html']:
            for url in re.findall(r'(?:href|src)\s*=\s*["\']([^"\']+)',t.get('raw','')):yield 'html',html.unescape(url)
        yield from links(t.get('children',[]))
def vendor(p):return 'managed_components' in p.parts or 'pro-api-sdk' in p.parts or 'stewart-core' in p.parts
paths=subprocess.check_output(['git','-C',str(ROOT),'ls-files','--cached','--others','--exclude-standard','-z']).decode().split('\0')
readmes={ROOT/p for p in paths if p and Path(p).name.lower().startswith('readme') and (ROOT/p).is_file()}
readmes.update((ROOT/'controller/stewart-core').glob('README*'))
rows=[];external={};acache={}
for p in sorted(readmes):
    for kind,url in links(markdown(p.read_text(encoding='utf-8-sig',errors='replace'))):
        row={'readme':p.relative_to(ROOT).as_posix(),'kind':kind,'target':url,'vendor':vendor(p)}
        u=urlsplit(url)
        if u.scheme in ['http','https']:
            row['status']='EXTERNAL_NOT_CHECKED'; external.setdefault(url.split('#')[0],[]).append(row)
        elif u.scheme in ['mailto','tel']:
            row['status']='ADDRESS_LINK'
        elif u.scheme:
            row['status']='NONPORTABLE_OR_UNKNOWN_SCHEME'
        else:
            target=(p.parent/unquote(u.path)).resolve() if u.path else p
            if not target.exists():row['status']='MISSING_LOCAL_TARGET'
            elif u.fragment and target.suffix.lower()=='.md':
                if target not in acache:acache[target]=anchors(target)
                row['status']='PASS' if unquote(u.fragment) in acache[target] else 'MISSING_LOCAL_ANCHOR'
            else:row['status']='PASS'
        rows.append(row)
def probe(url):
    request_url=quote(url,safe=":/?#[]@!$&'()*+,;=%")
    headers={'User-Agent':'Mozilla/5.0 (compatible; 6DOF documentation link audit)'}
    for method in ['HEAD','GET']:
        try:
            with urlopen(Request(request_url,method=method,headers=headers),timeout=12) as r:
                return url,{'status':'PASS_HTTP','httpStatus':r.status,'resolvedUrl':r.url}
        except HTTPError as e:
            if method=='HEAD':continue
            return url,{'status':'HTTP_UNVERIFIED' if e.code in [401,403,429] else 'HTTP_BROKEN','httpStatus':e.code}
        except (URLError,TimeoutError,OSError) as e:
            if method=='HEAD':continue
            return url,{'status':'HTTP_UNVERIFIED','error':type(e).__name__}
if args.external:
    with ThreadPoolExecutor(max_workers=8) as pool:
        for i,(url,result) in enumerate(pool.map(probe,external)):
            for row in external[url]:row.update(result)
            if (i+1)%20==0:print('HTTP checked',i+1,'/',len(external),flush=True)
bad=[r for r in rows if r['status'] in ['MISSING_LOCAL_TARGET','MISSING_LOCAL_ANCHOR','NONPORTABLE_OR_UNKNOWN_SCHEME','HTTP_BROKEN']]
unverified=[r for r in rows if r['status']=='HTTP_UNVERIFIED']
result={'checkedUtc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'readmes':len(readmes),'linkOccurrences':len(rows),'uniqueExternalUrls':len(external),'externalChecked':args.external,'brokenOwned':[r for r in bad if not r['vendor']],'brokenVendor':[r for r in bad if r['vendor']],'unverifiedHttp':unverified,'links':rows}
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='links'},indent=2,ensure_ascii=True))
raise SystemExit(bool(result['brokenOwned']))
