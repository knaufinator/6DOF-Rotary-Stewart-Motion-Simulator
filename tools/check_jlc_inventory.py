"""Read-only exact-code JLC inventory refresh for a normalized assembly BOM.

Uses public catalog search plus the product page's server data. The latter's
canPresaleNumber is the page's Available Order Qty, not its overseasStockCount.
No login, cart, quotation, order, reservation or design mutation is performed.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import re
import ssl
import time
import urllib.error
import urllib.request

SEARCH = 'https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList'
SEARCH_FIELDS = ('componentCode','componentModelEn','componentBrandEn','componentSpecificationEn',
                 'componentLibraryType','stockCount','canPresaleNumber','leastPatchNumber','lossNumber',
                 'minPurchaseNum','preMinPurchaseNum','encapsulationNumber','isBuyComponent','allowPostFlag',
                 'noBuyReason','componentAlternativesCode','mergedComponentCode','urlSuffix','initialPrice')
DETAIL_FIELDS = ('componentCode','componentModelEn','componentBrandEn','componentSpecificationEn',
                 'lcscComponentId','isBuyComponent','allowPostFlag','componentAlternativesCode','miniNumLimit',
                 'preMinPurchaseNum','encapsulationNumber','canPresaleNumber','overseasStockCount',
                 'initialPrice','lossNumber','leastPatchNumber','noBuyReason')


def utcnow():
    return datetime.now(timezone.utc).isoformat()


def detail_from_html(html, code):
    """Decode JSON flight records, never execute downloaded JavaScript."""
    chunks=[]
    for match in re.finditer(r'self\.__next_f\.push\((\[.*?\])\)</script>',html,re.S):
        item=json.loads(match.group(1))
        if len(item)>1 and isinstance(item[1],str):chunks.append(item[1])
    found=[]

    def visit(value):
        if isinstance(value,dict):
            if value.get('componentCode')==code and 'canPresaleNumber' in value:
                found.append({k:value.get(k) for k in DETAIL_FIELDS})
            for child in value.values():visit(child)
        elif isinstance(value,list):
            for child in value:visit(child)

    for line in ''.join(chunks).splitlines():
        try:record=json.loads(line.split(':',1)[1])
        except (ValueError,IndexError):continue
        visit(record)
    unique={json.dumps(v,sort_keys=True):v for v in found}
    if len(unique)!=1:
        raise ValueError(f'{code}: expected one consistent exact-code detail, found {len(unique)}')
    return next(iter(unique.values()))


def quantity(value):
    # bool, absent, negative and framework placeholders are unknown, never zero.
    return value if type(value) is int and value>=0 else None


def classify(detail, required):
    available=quantity(detail.get('canPresaleNumber'))
    warehouse=quantity(detail.get('overseasStockCount'))
    if detail.get('isBuyComponent')=='0':return 'PUBLIC_NOT_BUYABLE'
    if detail.get('isBuyComponent')!='1':return 'UNKNOWN'
    if available is None or warehouse is None:return 'UNKNOWN'
    if available<required:return 'PUBLIC_ORDER_QTY_SHORTFALL'
    if warehouse==0:return 'PREORDER_OR_SOURCING_REQUIRED'
    return 'PUBLIC_NET_QUANTITY_COVERED'


def read_bom(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as stream:
        rows=list(csv.DictReader(stream))
    refs=set();codes=set()
    for row in rows:
        code=row['LCSC Part #'].strip()
        if not re.fullmatch(r'C\d+',code) or code in codes:
            raise ValueError('Invalid or duplicate supplier code: '+code)
        designators=[r.strip() for r in row['Designator'].split(',') if r.strip()]
        if not designators or len(designators)!=int(row['Quantity']):
            raise ValueError('BOM quantity/designator mismatch: '+code)
        if len(set(designators))!=len(designators) or refs.intersection(designators):
            raise ValueError('Duplicate BOM designator: '+code)
        refs.update(designators);codes.add(code)
    return rows,refs


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bom',type=Path,required=True)
    parser.add_argument('--cpl',type=Path)
    parser.add_argument('--boards',type=int,default=5)
    parser.add_argument('--workers',type=int,default=3)
    parser.add_argument('--extra-code',action='append',default=[])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.boards<1 or not 1<=args.workers<=4:parser.error('Positive board count and 1..4 workers required')
    rows,refs=read_bom(args.bom)
    if args.cpl:
        with args.cpl.open(encoding='utf-8-sig',newline='') as stream:
            placements=list(csv.DictReader(stream))
        names=[r['Designator'].strip() for r in placements]
        if len(names)!=len(set(names)) or set(names)!=refs:raise ValueError('BOM/CPL reference mismatch')
    try:
        import certifi
        context=ssl.create_default_context(cafile=certifi.where())
    except ImportError:context=ssl.create_default_context()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)

    def fetch(url, payload=None):
        request=urllib.request.Request(url,data=None if payload is None else json.dumps(payload).encode(),
            headers={'User-Agent':'6DOF-readonly-inventory/1.0','Cache-Control':'no-cache','Content-Type':'application/json'})
        for attempt in range(2):
            try:
                with urllib.request.urlopen(request,context=context,timeout=25) as response:
                    return response.read(),{'date':response.headers.get('Date'),'age':response.headers.get('Age')}
            except urllib.error.HTTPError as error:
                if attempt or error.code not in (429,500,502,503,504):raise
                delay=error.headers.get('Retry-After','2')
                time.sleep(min(30,int(delay)) if delay.isdigit() else 2)
        raise RuntimeError('Fetch retry exhausted')

    def check(row):
        code=row['LCSC Part #'].strip()
        item={'code':code,'checkedUtc':utcnow(),'designators':row.get('Designator',''),
              'bomComment':row.get('Comment',''),'bomFootprint':row.get('Footprint',''),
              'perBoard':int(row.get('Quantity',0)),'criticality':'REQUIRED' if row.get('Designator') else 'UNAPPLIED_CANDIDATE'}
        item['requiredFor2Boards']=item['perBoard']*2
        item['requiredForBatch']=item['perBoard']*args.boards
        try:
            payload,headers=fetch(SEARCH,{'keyword':code,'currentPage':1,'pageSize':20})
            body=json.loads(payload)
            candidates=body.get('data',{}).get('componentPageInfo',{}).get('list',[]) or []
            exact=[r for r in candidates if r.get('componentCode')==code]
            if len(exact)!=1:raise ValueError(f'{code}: exact catalog match count {len(exact)}')
            catalog={k:exact[0].get(k) for k in SEARCH_FIELDS}
            item['catalog']=catalog;item['catalogResponseHeaders']=headers
            suffix=catalog.get('urlSuffix')
            if not suffix or not suffix.endswith('/'+code):raise ValueError('Unverified product URL suffix')
            item['url']='https://jlcpcb.com/partdetail/'+suffix
            html,page_headers=fetch(item['url'])
            item['pageResponseHeaders']=page_headers
            item['pageSha256']=hashlib.sha256(html).hexdigest()
            detail=detail_from_html(html.decode('utf-8'),code)
            item['detail']=detail;item['checkedUtc']=utcnow()
            item['availableOrderQty']=quantity(detail.get('canPresaleNumber'))
            item['catalogStockQty']=quantity(detail.get('overseasStockCount'))
            item['status']=classify(detail,item['requiredForBatch']) if item['perBoard'] else 'UNAPPLIED_CANDIDATE'
            item['netShortfall']=None if item['availableOrderQty'] is None else max(0,item['requiredForBatch']-item['availableOrderQty'])
        except Exception as error:
            item['status']='UNKNOWN';item['error']=str(error)
        return item

    extra=[{'LCSC Part #':code} for code in args.extra_code]
    started=utcnow();results=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(check,r) for r in rows+extra]):
            item=future.result();results.append(item)
            if item['status']!='PUBLIC_NET_QUANTITY_COVERED':
                print(json.dumps({k:item.get(k) for k in ('code','designators','status','requiredForBatch','availableOrderQty','catalogStockQty','error')}),flush=True)
    by_code={r['code']:r for r in results}
    ordered=[by_code[r['LCSC Part #']] for r in rows]
    statuses={status:sum(r['status']==status for r in ordered) for status in sorted({r['status'] for r in ordered})}
    status=('PUBLIC_SHORTFALLS_FOUND' if any(s in statuses for s in ('PUBLIC_ORDER_QTY_SHORTFALL','PUBLIC_NOT_BUYABLE'))
            else 'INCOMPLETE_PUBLIC_CHECK' if 'UNKNOWN' in statuses
            else 'PUBLIC_PREORDER_REQUIRED' if 'PREORDER_OR_SOURCING_REQUIRED' in statuses
            else 'PUBLIC_NET_REQUIREMENTS_COVERED_ALLOCATION_UNVERIFIED')
    report={'status':status,'startedUtc':started,'completedUtc':utcnow(),
            'bom':args.bom.as_posix(),'bomSha256':hashlib.sha256(args.bom.read_bytes()).hexdigest(),
            'boardQuantity':args.boards,'boardQuantityBasis':'Provisional screen; actual order quantity and attrition must be confirmed',
            'partCount':len(refs),'supplierCodes':len(rows),'allPartsRequired':True,'publicStatusCounts':statuses,
            'searchEndpoint':SEARCH,'quantityField':'Product page server data canPresaleNumber (Available Order Qty)',
            'catalogStockField':'Product page overseasStockCount; separate from Available Order Qty',
            'quantityPolicy':'Compare net fitted demand only; no invented JLC attrition or account-private allocation',
            'privateInventoryChecked':False,'stockReserved':False,'items':ordered,'unappliedCandidates':[by_code[r['LCSC Part #']] for r in extra]}
    if args.cpl:report['cplSha256']=hashlib.sha256(args.cpl.read_bytes()).hexdigest()
    (out/'INVENTORY.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    columns=['code','designators','perBoard','requiredFor2Boards','requiredForBatch','availableOrderQty','catalogStockQty',
             'netShortfall','status','model','manufacturer','libraryType','minimumPurchase','catalogLossNumber','url','checkedUtc']
    with (out/'INVENTORY.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        for item in ordered:
            detail=item.get('detail',{});catalog=item.get('catalog',{})
            output={k:item.get(k) for k in columns}
            output.update(model=detail.get('componentModelEn'),manufacturer=detail.get('componentBrandEn'),
                          libraryType=catalog.get('componentLibraryType'),minimumPurchase=catalog.get('minPurchaseNum'),
                          catalogLossNumber=detail.get('lossNumber'))
            writer.writerow(output)
    print(json.dumps({'supplierCodes':len(rows),'parts':len(refs),'publicStatusCounts':statuses,'output':str(out)},indent=2))


if __name__=='__main__':main()
