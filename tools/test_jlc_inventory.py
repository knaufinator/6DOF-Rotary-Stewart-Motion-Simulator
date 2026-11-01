"""Offline checks for stock-field confusion and untrusted catalog data."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

from check_jlc_inventory import classify,detail_from_html,quantity,read_bom


class InventoryTests(unittest.TestCase):
    def detail(self,order=0,stock=65,buy='1'):
        return {'componentCode':'C77833','componentModelEn':'DS1034-25FUNSI44',
                'canPresaleNumber':order,'overseasStockCount':stock,'isBuyComponent':buy}

    def flight(self,*records):
        payload=''.join(f'{n:x}:'+json.dumps(record)+'\n' for n,record in enumerate(records))
        return '<script>self.__next_f.push('+json.dumps([1,payload])+')</script>'

    def test_warehouse_stock_cannot_hide_zero_order_quantity(self):
        self.assertEqual(classify(self.detail(),30),'PUBLIC_ORDER_QTY_SHORTFALL')

    def test_large_order_quantity_covers_net_demand(self):
        self.assertEqual(classify(self.detail(order=15666,stock=16263),5),'PUBLIC_NET_QUANTITY_COVERED')

    def test_empty_warehouse_is_not_immediate_assembly_stock(self):
        self.assertEqual(classify(self.detail(order=100,stock=0),5),'PREORDER_OR_SOURCING_REQUIRED')

    def test_unknown_values_do_not_become_stock(self):
        for value in [None,-666,'$undefined','65',True]:
            self.assertIsNone(quantity(value))
            self.assertEqual(classify(self.detail(order=value),5),'UNKNOWN')
        self.assertEqual(quantity(0),0)

    def test_buyability_requires_explicit_state(self):
        self.assertEqual(classify(self.detail(order=100,buy='0'),5),'PUBLIC_NOT_BUYABLE')
        self.assertEqual(classify(self.detail(order=100,buy=None),5),'UNKNOWN')

    def test_exact_identity_ignores_related_recommendations(self):
        related=dict(self.detail(order=1000000),componentCode='C99999')
        exact=self.detail()
        result=detail_from_html(self.flight({'children':[related,exact,exact]}),'C77833')
        self.assertEqual(result['canPresaleNumber'],0)
        self.assertEqual(result['componentCode'],'C77833')

    def test_conflicting_page_records_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'consistent exact-code'):
            detail_from_html(self.flight(self.detail(),self.detail(order=100)),'C77833')

    def test_missing_page_identity_is_rejected(self):
        with self.assertRaises(ValueError):detail_from_html(self.flight(self.detail()),'C12345')

    def test_duplicate_refs_and_quantity_mismatch_rejected(self):
        for refs,qty in [('R1,R1',2),('R1,R2',1)]:
            with tempfile.TemporaryDirectory() as temp:
                path=Path(temp)/'bom.csv'
                with path.open('w',newline='') as stream:
                    writer=csv.writer(stream);writer.writerow(['LCSC Part #','Designator','Quantity'])
                    writer.writerow(['C123',refs,qty])
                with self.assertRaises(ValueError):read_bom(path)


if __name__=='__main__':unittest.main()
